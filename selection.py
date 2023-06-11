from PIL import Image
import redis
import psutil
import subprocess
# creating a object
import signal

pool = redis.ConnectionPool(host='localhost', port=6379, db=0)
redis = redis.Redis(connection_pool=pool)
redis.set('pose::complete', "0")
im = Image.open(r"poses.png")
im.show()


while True:
    
    # viewer = subprocess.Popen(['python','imviewer.py'])
    redis.set('pose::selection', 0)
    pose = input("Select a pose (input a number 1-5): ")
    # print(pose)
    redis.set('pose::selection', pose)

    # viewer.terminate()
    # viewer.kill() 
    # viewer.send_signal(signal.SIGTERM)
    print("You selected pose", pose)
    print("Try to hit the pose now...")
    while redis.get('pose::complete') == b'0':
        pass


