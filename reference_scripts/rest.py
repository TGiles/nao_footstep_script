import naoqi
from naoqi import ALProxy
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--ip', type=str, help='Robot IP address')
args = parser.parse_args()
motion = ALProxy('ALMotion', args.ip, 9559)
motion.rest()
print 'Resting robot'