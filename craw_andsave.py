#coding:utf-8
'''
这次只爬取图片标签，然后怎么保存到本地的文件夹中
用到requests.urltrieve
'''
from bs4 import BeautifulSoup
import requests
import time
import urllib

url_links=[]
for num in range(180,201):
    url = "http://jandan.net/ooxx/page-{}#comments".format(str(num))
    url_links.append(url)

holder="F://pycharm_workspace/"

def hihihi(url):
    headers={
        'User-Agent':"Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/59.0.3071.115 Safari/537.36",
        'Cookie':"jdna=01b0531fab6a989460dd1b231010b496#1500868101349; _ga=GA1.2.329888517.1500865062; _gid=GA1.2.1475003807.1500865062"
    }
    wb_data=requests.get(url,headers=headers)
    soup=BeautifulSoup(wb_data.text,'lxml')
    pics = soup.find_all('img')
    for pic in pics:
        picadd=pic.get('src')
        urllib.urlretrieve(picadd,holder+picadd[-10:])

for urloo in url_links:
    hihihi(urloo)









