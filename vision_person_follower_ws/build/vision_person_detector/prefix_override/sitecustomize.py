import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ashvin/vision_person_follower_project/vision_person_follower_ws/install/vision_person_detector'
