#!/usr/bin/python
import requests
import time

# The base URL is the one specified through: http://192.168.2.2:2770/waterlinked
base_url = 'http://192.168.2.94:80/api/v1'

# The complete API can be found here: http://192.168.2.94/swagger/
api_list = ['/about', '/about/status', '/about/temperature', '/position/master',
            '/config/generic', '/config/receivers',
            '/external/orientation',
            '/position/acoustic/filtered', '/position/acoustic/raw', '/position/global', '/position/master']

urls = [base_url + api for api in api_list]

def main_func():

  n = 0
  while True:
    resp_list = [requests.get(url) for url in urls]
    print(n)
    for resp in resp_list:
       print resp.content
       f = open("GPS-loggin.txt",'a')
       f.write(str(resp.content))
       f.close()
    n += 1

if __name__ == '__main__':
    main_func()
