#! /usr/bin/python
import chess.uci
import rospy
from skimage import color
from scipy.ndimage import imread
import numpy as np
import matplotlib.pyplot as plt
from skimage.feature import hog
import pickle
import glob
import argparse
import cv_bridge
import cv2 as v
from scipy.misc import imsave

from project.msg import BoardMessage, MoveMessage
from geometry_msgs.msg import Point

PIXEL_SIZE = 256 #Read from images
grid = (-1)**np.mgrid[0:8,0:8].T.reshape((-1,2)).sum(axis=1).flatten()

piece_heights = {}
ph = {'p':0.14, 'r':0.14, 'b':0.14, 'n':0.17, 'q':0.17, 'k':0.17}
for key in ph:
    piece_heights[key] = piece_heights[key.upper()] = ph[key] + 0.3
del ph


PLAYING = None

std_ordering = np.array(chess.SQUARES).reshape((8,8))[::-1,:].flatten()


def initialize(image):
    # Figure out which side Baxter is playing
    global PLAYING, prev_board, ordering
    PLAYING = determine_initial_state(image)
    if PLAYING == "WHITE":
        prev_board = np.array([2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,\
                                0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,\
                                0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,\
                                1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1])
        ordering = std_ordering
    elif PLAYING == "BLACK":
        prev_board = np.array([1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,\
                                0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,\
                                0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,\
                                2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2])
        ordering = std_ordering[::-1]
    elif PLAYING in ("wb", "bw"):
        PLAYING = None


def split_image(image):
    delta = PIXEL_SIZE/8
    squares = []
    for i in range(8):
            x = i*delta
            for j in range(8):
                y = j*delta
                squares.append(image[x:x+delta, y:y+delta])
    return squares

def standardize_evidence(evidence):
    if PLAYING == 'BLACK':
        evidence.reverse()
    return evidence

def squareid_to_coord(square):
    rank = chess.rank_index(square)
    file = chess.file_index(square)
    
    if PLAYING == 'BLACK':
        rank = 7 - rank
    elif PLAYING == 'WHITE':
        file = 7 - file

    return np.array([rank+0.5, file+0.5, 1])

def get_squaremap(corners):
    d1t2 = (corners[2] - corners[1])/8.0
    d1t3 = (corners[3] - corners[1])/8.0
    return np.vstack((d1t2,d1t3,corners[0])).T

def determine_initial_state(image):
    evidence, __, __ = detect_pieces(image)
    states = {"WHITE":np.concatenate([np.ones(16) + 1, np.zeros(32), np.ones(16)]),\
                "BLACK": np.concatenate([np.ones(16), np.zeros(32), np.ones(16)+1]),\
                "wb":np.array([1,1,0,0,0,0,2,2,\
                            1,1,0,0,0,0,2,2,\
                            1,1,0,0,0,0,2,2,\
                            1,1,0,0,0,0,2,2,\
                            1,1,0,0,0,0,2,2,\
                            1,1,0,0,0,0,2,2,\
                            1,1,0,0,0,0,2,2,\
                            1,1,0,0,0,0,2,2]),\
                "bw":np.array([2,2,0,0,0,0,1,1,\
                            2,2,0,0,0,0,1,1,\
                            2,2,0,0,0,0,1,1,\
                            2,2,0,0,0,0,1,1,\
                            2,2,0,0,0,0,1,1,\
                            2,2,0,0,0,0,1,1,\
                            2,2,0,0,0,0,1,1,\
                            2,2,0,0,0,0,1,1,])}
    for state in states:
        states[state] = compute_score(states[state], evidence)
    return max(states, key=states.get)


def compute_score(board, prob_table):
    if type(board) != np.ndarray:
        board = board_to_mask(board)
    prob = 0
    for i in range(len(board)):
        a = prob_table[int(board[i]),i]
        prob += np.log(a)
    return prob

def featurize(image):
    image = image[4:28, 4:28]
    #return np.append(image.flatten(), np.std(image))
    ret = np.append(hog(image, orientations=8, pixels_per_cell=(8,8),\
           cells_per_block=(1,1), visualise=False), np.std(image)).reshape(1,-1)
    return ret


def detect_pieces(image):
    """
    Return probability distribution of 
    pieces on the board
    """
    squares = standardize_evidence(split_image(image))
    featurized = []
    i = 0
    for square in squares:
        featurized.append(featurize(square))
    cur_board = []
    for square in featurized:
        p = brain.predict_proba(square)
        cur_board.append(brain.predict_proba(square).reshape((3,1)))
        i += 1
    cur_board = np.hstack(cur_board)
    return cur_board

def detect_pieces(image):
    squares = standardize_evidence(split_image(image))
    featurized = np.zeros((64,73))
    for i in xrange(64):
        featurized[i,:] = featurize(squares[i])
    proba = brain.predict_proba(featurized).T
    return proba, np.argmax(proba, axis=0).astype(int), squares

def board_to_mask(board):
    """
    Turns a Chess.Board into an array of
    empty, white, black array
    """
    mask = np.zeros((64,))
    for square in chess.SQUARES:
        piece = board.piece_at(square)
        if piece == None:
            piece = 0
        elif piece.color == True:    
            piece = 1
        else:
            piece = 2
        mask[std_ordering[square]] = piece
    return mask.astype(int)

minf = float('-inf')
def most_prob_state(evidence, board):
    board = board.copy()
    biggest = False, compute_score(board, evidence)
    second = None, minf
    for move in board.legal_moves:
        board.push(move)
        prob = compute_score(board, evidence)
        if prob > biggest[1]:
            second = biggest
            biggest = move, prob
        elif prob > second[1]:
            second = move, prob
        board.pop()

    print biggest, second

    epsilon = 0.1
    if not biggest[1] - second[1] >= epsilon:
        print 'Not a big enough difference'
        return board, None
    if biggest[0] is False:
        return board, False
    board.push(biggest[0])
    return board, biggest[0]
            
        

def most_prob_state_old(evidence, board):
    #apppend no move and save prev_move
    #No move logic
    moves = list(board.legal_moves)
    boards = [board.copy() for i in range(len(moves))]
    for i in range(len(boards)):
        boards[i].push(moves[i])
    boards.append(board.copy())
    moves.append(False)
    probs = []
    for b in boards:
        probs.append(compute_score(b, evidence))
    probs = np.array(probs)
    ind = np.argsort(-probs)
    print(probs[ind[0]], probs[ind[1]])

    #Thresholding
    epsilon = 0.0
    print('Probability difference: %f' % abs(probs[ind[0]] - probs[ind[1]]))
    print moves[ind[0]], moves[ind[1]]
    if not abs(probs[ind[0]] - probs[ind[1]]) >= epsilon:
        print("Not a big enough difference")
        return board, None
    return boards[ind[0]], moves[ind[0]]

bridge = cv_bridge.CvBridge()

def unmake_point_message(pt):
    return np.array([pt.x, pt.y, pt.z])

prev_image = None
last_turn = -1
BOARD_DIFFERENCE_THRESHOLD = 700000
def callback(data):
    global board, prev_image, A, last_turn

    since = rospy.Time.now() - data.unperspective.header.stamp
    if since.to_sec() > 0.25:
        return

    points = [data.topleft, data.topright, data.botleft, data.botright]
    points = map(unmake_point_message, points)
    A = get_squaremap(points)

    image = bridge.imgmsg_to_cv2(data.unperspective, desired_encoding='bgr8')
    image = v.cvtColor(image, v.COLOR_BGR2GRAY)

    if prev_image is None:
        prev_image = image
        return

    prev_image = image
    if ((prev_image - image)**2).sum() > BOARD_DIFFERENCE_THRESHOLD:
        print "The board isn't stable"
        return

    if PLAYING == None:
        initialize(image)
        print 'We are playing as ', PLAYING
        return

    evidence, labels, squares = detect_pieces(image)
    board, move = most_prob_state(evidence, board)
    b = board_to_mask(board)
    ind = np.where(b != labels)[0]
    print(ind)
    print board

    #Data gathering
    if SAVE_MLE_ERRORS or SAVE_INCORRECT:
        cor = raw_input("Was this board guess correct? [y/n]:")
        if cor == 'y' and SAVE_MLE_ERRORS:
            p = '/home/shallowblue/Project/src/project/src/Correct/'
            through = ind
            for j in through:
                prepend = p
                if brain.mode == 'difference':
                    if b[j] == 1:
                        if grid[j] == 1:
                            prepend += "WhiteOnWhite/"
                        else:
                            prepend += "WhiteOnBlack/"
                    elif labels[j] == 2:
                        if grid[j] == 1:
                            prepend += "BlackOnBlack/"
                        else:
                            prepend += "BlackOnWhite/"
                    elif labels[j] == 0:
                        if grid[j] == 1:
                            prepend += "EmptyWhite/"
                        else:
                            prepend += "EmptyBlack/"            
                elif brain.mode == 'color':
                    if b[j] == 0:
                        prepend += "Empty"
                    elif b[j] == 1:
                        prepend += "WhiteOn"
                    else:
                        prepend += "BlackOn"
                    if grid[j] == 1:
                        prepend += "White/"
                    else:
                        prepend += "Black/"
                imsave(prepend + str(data.unperspective.header.stamp)\
                                     +'-'+ str(j)+'.jpg',\
                                 squares[j])
        elif SAVE_INCORRECT:
            p = '/home/shallowblue/Project/src/project/src/Incorrect/'
            for j in ordering:
              imsave('{}-{}.jpg'.format(data.unperspective.header.stamp,j),
                     squares[j])
            
    if board.is_game_over():
        print '## GAME OVER, HUMAN!'
        rospy.signal_shutdown('Because the game is over')

    if move is None:
        print 'No best board! Something went wrong.'
        # request perturbation

    if board.turn == (PLAYING == 'WHITE') and board.fullmove_number > last_turn:
        last_turn = board.fullmove_number
        engine.position(board)
        reply = engine.go(movetime=500, async_callback=False).bestmove
        make_move(reply)


def create_move_msg(strt, dest, piece, type=0):
    msg = MoveMessage()
    ofs = np.array([0, 0, piece_heights[piece]])
    msg.source = make_point_message(strt + ofs)
    msg.destination = make_point_message(dest + ofs)
    msg.type = type
    return msg


def make_move(move):
    print '## {}'.format(board.san(move))
    
    # find real-world start & end positions 
    start = A.dot(squareid_to_coord(move.from_square))
    end = A.dot(squareid_to_coord(move.to_square))

    # if we've captured, first send a message saying to remove that piece
    square_contents = board.piece_at(move.to_square)
    if square_contents != None:
        square_contents = square_contents.symbol()
        pub.publish(create_move_msg(end, end, square_contents, 1))

    # create the message
    square_contents = board.piece_at(move.from_square).symbol()
    pub.publish(create_move_msg(start, end, square_contents, 0))




def make_point_message(pt):
    ret = Point()
    ret.x, ret.y, ret.z = pt
    return ret
        
if __name__ == '__main__':
    rospy.init_node('brain')

    desc = "ROS node that takes the Eye's BoardMessages and BRAINS!!"
    parser = argparse.ArgumentParser(description = desc)
    parser.add_argument('-i', '--input', 
                        help='input BoardMessage topic')
    parser.add_argument('-o', '--output',
                        help='output MoveMessage topic')
    parser.add_argument('-e', '--engine', default='stockfish',
                        help='executable of engine to play with')
    parser.add_argument('-b', '--brain', default='pickled_brain.p')
    parser.add_argument('--save-incorrects', action='store_true')
    parser.add_argument('--save-mle-errors', action='store_true')
    args = parser.parse_args(rospy.myargv()[1:])

    SAVE_INCORRECT = args.save_incorrects
    SAVE_MLE_ERRORS = args.save_mle_errors

    brain = pickle.load(open(args.brain, 'rb'))
    engine = chess.uci.popen_engine(args.engine)
    engine.uci()
    board = chess.Board()
    prev_board = []

    pub = rospy.Publisher(args.output, MoveMessage, latch=True, queue_size=10)

    rospy.Subscriber(args.input, BoardMessage, callback)
    print 'Frontal cortex ready!'
    rospy.spin()

