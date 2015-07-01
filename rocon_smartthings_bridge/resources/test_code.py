#!/usr/bin/env python

import json
import requests

def load_settings(filename="smartthings.json"):
    """Load the JSON Settings file. 
    
    See the documentation, but briefly you can
    get it from here:
    https://iotdb.org/playground/oauthorize
    """
    with open(filename) as fin:
        std = json.load(fin)
    return std['api'], std['api_location'], std['access_token']

def get_endpoint(url, api_location, access_token):
    endpoints_url = url 
    endpoints_paramd = {
        "access_token": access_token
    }

    endpoints_response = requests.get(url=endpoints_url, params=endpoints_paramd)
    end_url = endpoints_response.json()[0]['url']
    endpoint_url = 'http://%s%s'%(api_location, end_url)
    return endpoint_url

def request_get(url, access_token, command):
    request_url = "%s/%s"%(url, command)
    params = {}
    header =  {
      "Authorization": "Bearer %s" % access_token,
    }
    resp = requests.get(url=request_url, params=params, headers=header)
    r = resp.json()

    if type(r) == list:
        r = r[0]

    if type(r) == dict:
        for k, v in r.items():
            print("--- %s"%str(k))
            if type(v) == list or type(v) == dict:
                if v:
                    for kk in v:
                        for kkk, vvv in kk.items():
                            print("\t%s : %s"%(kkk, vvv))
                        print("")
            else:
                print(str("    %s"%v))
    else:
        print(str(r))



def request_to_update_uri(url, access_token, paired_uri):
    request_url = "%s/%s"%(url, "configuration")
    params = {"uri": paired_uri}
    header =  {
      "Authorization": "Bearer %s" % access_token,
    }
    resp = requests.put(url=request_url, params=params, headers=header)
    print(str(resp))

if __name__ == '__main__':
    api, api_location, access_token = load_settings()
    endpoint_url = get_endpoint(api, api_location, access_token)
    #request_devices(endpoint_url, access_token, "motion")
    #r = request_get(endpoint_url, access_token,"get_all_types")
    #r = request_get(endpoint_url, access_token,"configuration")
    r = request_to_update_uri(endpoint_url, access_token, "qiowjefqojiwefoqijfqwe")
    r = request_get(endpoint_url, access_token,"configuration")
