#!/usr/bin/python
import rospy
import grequests
import time
#import sys

# The base URL is the one specified through: http://192.168.2.2:2770/waterlinked
base_url = 'http://192.168.2.94:80/api/v1'

# The complete API can be found here: http://192.168.2.94/swagger/
api_list = ['/about', '/about/status', '/about/temperature', '/position/master',
            '/config/generic', '/config/receivers',
            '/external/orientation',
            '/position/acoustic/filtered', '/position/acoustic/raw', '/position/global', '/position/master']

urls = [base_url + api for api in api_list]
#print urls

def main_func():
  rospy.init_node('waterlinked_gps_node', anonymous=True)
  rate = rospy.Rate(50)

  t0 = time.time()
  is_first_iteration = True
  f_cum = 0
  n = 0

  rs = [grequests.get(url) for url in urls]

  while not rospy.is_shutdown():

    # Get timing
    tnow = time.time()
    if is_first_iteration:
        is_first_iteration = False
    else:
        f = 1/(tnow-t0)
        f_cum += f
        n+=1
        f_avg = f_cum / n
        print('n = ' + str(n) + ', f_avg = ' + str(f_avg))
    t0 = tnow

    res=grequests.map(rs)
    for resp in res:
      print resp.content


    #url = "http://192.168.2.94:80/api/v1/position/master"
    #r = requests.get(url)
    #print(url)
    #print r.content
    #print('loop')
    rate.sleep()

if __name__ == '__main__':
  try:
    main_func()
  except rospy.ROSInterruptException:
    pass
