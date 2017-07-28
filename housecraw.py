#coding:utf-8
'''
爬取网站：http://bj.xiaozhu.com/search-duanzufang-p1-0/
主要采集信息为房子的：
1.标题
2.地址
3.日租金
4.第一张图片的链接
5.房东图片链接
6.房东性别（用if-else)
7.房东姓名
先爬第一页，然后爬300页信息（构建函数完成）
'''
from bs4 import  BeautifulSoup
import requests
import json
import  time
'''
爬取的是单一的网页，soup.select()选出的是列表，所以用【0】来选取第一个
'''

url="http://bj.xiaozhu.com/fangzi/2803985763.html"
wb_data=requests.get(url)
soup=BeautifulSoup(wb_data.text,'lxml')

titles=soup.select("div.pho_info > h4")[0]
addresses=soup.select("div.pho_info > p")[0]
moneys=soup.select("div.day_l > span")[0]
firstpics=soup.select("#curBigImage")[0]
landlordpics=soup.select("div.member_pic > a > img")[0]
landlordsexs=soup.select("div.member_pic > div")[0]
landlordnames=soup.select("a.lorder_name")[0]

'''print(titles.text)
print(addresses.get('title'))
print(moneys.text)
print(firstpics.get('src'))
print(landlordpics.get('src'))
print(landlordsexs.get('class')[0])
print(landlordnames.text)'''

#构造函数
def housecraw(url):
    wb_data = requests.get(url)
    soup = BeautifulSoup(wb_data.text, 'lxml')

    titles = soup.select("div.pho_info > h4")[0]
    addresses = soup.select("div.pho_info > p")[0]
    moneys = soup.select("div.day_l > span")[0]
    firstpics = soup.select("#curBigImage")[0]
    landlordpics = soup.select("div.member_pic > a > img")[0]
    landlordsexs = soup.select("div.member_pic > div")[0]
    landlordnames = soup.select("a.lorder_name")[0]
    data={
         'title':titles.text,
        'address':addresses.get('title'),
         'money':moneys.text,
         'firstpic':firstpics.get('src'),
         'landlordpic':landlordpics.get('src'),
         'landlordsex':landlordsexs.get('class')[0],
          'landlordname':landlordnames.text
    }
    print json.dumps(data, encoding="UTF-8", ensure_ascii=False)


page_link = [] # <- 每个详情页的链接都存在这里，解析详情的时候就遍历这个列表然后访问就好啦~

def get_page_link(page_number):
    for each_number in range(1,page_number): # 每页24个链接,这里输入的是页码
        full_url = 'http://bj.xiaozhu.com/search-duanzufang-p{}-0/'.format(each_number)
        wb_data = requests.get(full_url)
        soup = BeautifulSoup(wb_data.text,'lxml')
        for link in soup.select('a.resule_img_a'): # 找到这个 class 样为resule_img_a 的 a 标签即可
            page_link.append(link.get('href'))

get_page_link(30)
for urlpage in page_link:
    time.sleep(2)
    housecraw(urlpage)