import requests
import time
import urllib
from bs4 import BeautifulSoup


url_link = "http://jandan.net/ooxx/page-199#comments"

holder="F://pycharm_workspace/"


headers={
        'User-Agent':"Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/59.0.3071.115 Safari/537.36",
        'Cookie':"jdna=01b0531fab6a989460dd1b231010b496#1500868101349; _ga=GA1.2.329888517.1500865062; _gid=GA1.2.1475003807.1500865062"
    }

wb_data=requests.get(url_link,headers=headers)
soup=BeautifulSoup(wb_data.text,'lxml')
downloadlinks=[]
pics = soup.find_all('img')
for pic in pics:
    picadd=pic.get('src')
    downloadlinks.append(picadd)

for item in downloadlinks:
    urllib.urlretrieve(item,holder+picadd[-10:])
    print('done!')



