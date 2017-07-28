import requests
import urllib
from bs4 import BeautifulSoup


url_link = "http://jandan.net/ooxx/page-199"

holder="F://pycharm_workspace/"
proxies = {"http": "http://121.69.29.162:8118"}

headers={
        'Host':"jandan.net",
        'User-Agent':'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/59.0.3071.115 Safari/537.36',
        'Accept':"text/html,application/xhtml+xml,application/xml;q=0.9,image/webp,image/apng,*/*;q=0.8",
        'Accept-Language':"en-US,en;q=0.8,zh-CN;q=0.6,zh;q=0.4",
        'Accept-Encoding':"gzip, deflate",
        'Referer':"http://jandan.net/ooxx",
        'Connection':'keep-alive',
        'Cache-Control':"max-age=0"}

wb_data=requests.get(url_link,proxies=proxies,headers=headers)
soup=BeautifulSoup(wb_data.text,'lxml')
downloadlinks=[]
pics = soup.find_all('img')
for pic in pics:
    picadd=pic.get('src')
    downloadlinks.append(picadd)

for item in downloadlinks:
    urllib.urlretrieve(item,holder+item[-5:])
    print('done!')

