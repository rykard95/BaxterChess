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

from project.msg import BoardMessage, MoveMessage

PIXEL_SIZE = 256 #Read from images

square_to_location = {'A7': None, 'A2': None, 'A6': None, 'E5': None,\
                        'D7': None, 'B7': None, 'F1': None, 'B3': None,\
                        'B5': None, 'H1': None, 'H4': None, 'B6': None,\
                        'F3': None, 'F4': None, 'B2': None, 'C7': None,\
                        'C2': None, 'F2': None, 'E2': None, 'A3': None,\
                        'A5': None, 'G8': None, 'E6': None, 'C8': None,\
                        'H3': None, 'E8': None, 'E7': None, 'G7': None,\
                        'F6': None, 'G1': None, 'D1': None, 'G3': None,\
                        'F8': None, 'H8': None, 'F5': None, 'D5': None,\
                        'E4': None, 'H2': None, 'H7': None, 'B4': None,\
                        'G5': None, 'B8': None, 'C5': None, 'G4': None,\
                        'D4': None, 'F7': None, 'C3': None, 'E1': None,\
                        'D8': None, 'C4': None, 'C6': None, 'A1': None,\
                        'G6': None, 'H6': None, 'A8': None, 'E3': None,\
                        'C1': None, 'D3': None, 'B1': None, 'A4': None,\
                        'H5': None, 'D6': None, 'D2': None, 'G2': None}

piece_heights = {}
ph = {'p':0.14, 'r':0.14, 'b':0.14, 'n':0.17, 'q':0.17, 'k':0.17}
for key in ph:
    piece_heights[key.upper()] = ph[key]
    piece_heights[key] = ph[key]
del ph
PLAYING = None

def create_ordering():
    ordering = []
    for j in range(8,0,-1):
        for i in "ABCDEFGH":
            ordering.append(i+str(j))
    return np.array(ordering)

std_ordering = create_ordering()
rvs_ordering = std_ordering[::-1]

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
    if PLAYING == "BLACK":
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

def update_locations(corners):
    """
    Put corners into a numpy array
    """
    d1t2 = (corners[2] - corners[1])/8.0
    d1t3 = (corners[3] - corners[1])/8.0
    x = np.vstack((d1t2,d1t3,corners[0])).T
    p = np.ones((64,3))
    p[:,:2] = np.mgrid[0:8,0:8].T.reshape((-1,2)) + .5
    y = p.dot(x.T)
    for i in range(len(ordering)):
        square_to_location[ordering[i]] = y[i]
    
def determine_initial_state(image):
    evidence = detect_pieces(image)
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
        a = prob_table[board[i]][i]
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
    plt.imshow(squares[0],cmap=plt.cm.gray)
    plt.pause(0.25)
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

def board_to_mask(board):
    """
    Turns a Chess.Board into an array of
    empty, white, black array
    """
    mask = []
    for square in std_ordering:
        square =  getattr(chess, square)
        piece = board.piece_at(square)
        if piece == None:
            piece = 0
        elif piece.color == True:    
            piece = 1
        else:
            piece = 2
        mask.append(piece)
    return np.array(mask)

def most_prob_state(evidence, board):
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

def callback(data):
    global board

    points = [data.topleft, data.topright, data.botleft, data.botright]
    image = bridge.imgmsg_to_cv2(data.unperspective, 
                                 desired_encoding='bgr8')
    image = v.cvtColor(image, v.COLOR_BGR2GRAY)
    if PLAYING == None:
        initialize(image)
        print 'We are playing as ', PLAYING
        return

    evidence = detect_pieces(image)
    board, move = most_prob_state(evidence, board)

    if move is None:
        print 'No best board! Something went wrong.'
        # request perturbation
    elif move is False:
        print 'No move was made.'
        # don't do anything
    else:
        # get move from Stockfish & apply it to a board
        engine.position(board)
        reply = engine.go(movetime=500).bestmove

        # find real-world start & end positions 
        A = get_squaremap(points)
        start = A.dot(squareid_to_coord(reply.from_square))
        end = A.dot(squareid_to_coord(reply.to_square))

        # if we've captured, first send a message saying to remove that piece
        square_contents = board.piece_at(reply.to_square)
        if square_contents != None:
            msg = MoveMessage()
            ofs = np.array([0, 0, piece_heights[square_contents]])
            msg.source = msg.destination = make_point_message(end + ofs)
            msg.type = 1
            pub.publish(msg)

        # create the message
        square_contents = board.piece_at(reply.from_square)
        ofs = np.array([0, 0, piece_heights[square_contents]])
        msg = MoveMessage()
        msg.source = make_point_message(start + ofs)
        msg.destination = make_point_message(end + ofs)
        msg.type = 0
        pub.publish(msg)

        board.push(reply)

def make_point_message(pt):
    ret = Point()
    ret.x, ret.y, ret.z = pt
    return ret
        
if __name__ == '__main__':
    desc = "ROS node that takes the Eye's BoardMessages and BRAINS!!"
    parser = argparse.ArgumentParser(description = desc)
    parser.add_argument('-i', '--input', 
                        help='input BoardMessage topic')
    parser.add_argument('-o', '--output',
                        help='output MoveMessage topic')
    parser.add_argument('-e', '--engine', default='src/stockfish',
                        help='executable of engine to play with')
    parser.add_argument('--node-name', default='brain')
    parser.add_argument('-b', '--brain', default='src/pickled_brain.p')
    args = parser.parse_args()

    brain = pickle.load(open(args.brain, 'rb'))

    rospy.init_node(args.node_name)

    engine = chess.uci.popen_engine(args.engine)
    engine.uci()
    board = chess.Board()
    prev_board = []

    pub = rospy.Publisher(args.output, MoveMessage, latch=True, queue_size=10)

    rospy.Subscriber(args.input, BoardMessage, callback)
    print 'Frontal cortex ready!'
    rospy.spin()

