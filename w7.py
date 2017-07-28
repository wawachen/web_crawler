#coding:utf-8
from bs4 import BeautifulSoup
import requests
import json #为了让字典能够显示中文的title

url="https://www.tripadvisor.cn/Attractions-g60763-Activities-New_York_City_New_York.html#ATTRACTION_SORT_WRAPPER"
wb_data=requests.get(url)#利用requests 库get命令去解析url
soup=BeautifulSoup(wb_data.text,"lxml")#用beautifulsoup
titles=soup.select('div.listing_title > a[target="_blank"]')#soup.select去选择css selector 的位置
imgs=soup.select('img[width="180"]')#用属性去定位 img[width=""]
cates=soup.select("div.p13n_reasoning_v2")

for title,img,cate in zip(titles,imgs,cates):#zip 多重循环构造的向量
    data={
        'title':title.get_text(),#get_text获取文本形式
        'img':img.get("src"),
        'cate':list(cate.stripped_strings)#获取所有副级标签的文本stripped_strings
    }
    print json.dumps(data, encoding="UTF-8", ensure_ascii=False)#固定的格式

'''
为了方便也可以构建函数来方便实现多次重复爬取的功能
'''

url=["https://www.tripadvisor.cn/Attractions-g60763-Activities-oa{}-New_York_City_New_York.html#ATTRACTION_LIST".format(str(i) for i in range(30,930,30))]
def wbcrawler(url):
    wb_data = requests.get(url)  # 利用requests 库get命令去解析url
    soup = BeautifulSoup(wb_data.text, "lxml")  # 用beautifulsoup
    titles = soup.select('div.listing_title > a[target="_blank"]')  # soup.select去选择css selector 的位置
    imgs = soup.select('img[width="180"]')  # 用属性去定位 img[width=""]
    cates = soup.select("div.p13n_reasoning_v2")

    for title, img, cate in zip(titles, imgs, cates):  # zip 多重循环构造的向量
        data = {
            'title': title.get_text(),  # get_text获取文本形式
            'img': img.get("src"),
            'cate': list(cate.stripped_strings)  # 获取所有副级标签的文本stripped_strings
        }
        print json.dumps(data, encoding="UTF-8", ensure_ascii=False)  # 固定的格式

for urlib in url:
    wbcrawler(urlib)
