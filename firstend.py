'''
此段代码无法运行，只供参考学习（原因是网页出现了变动）
'''
from bs4 import BeautifulSoup
import requests

url="http://zhuanzhuan.58.com/detail/890462879296307209z.shtml?fullCate=5%2C38484%2C23094&fullLocal=858&from=pc&PGTID=0d305a36-0035-a7ac-3885-5185494cf94a&ClickID=1"
wb_data=requests.get(url)
soup=BeautifulSoup(wb_data.text,'lxml')

title=soup.title.text
price=soup.select('#content span.price')   # ‘#’井号是id 的意思
date=soup.select('.time')                   #  “.”点号是属性 class的意思
area=soup.select('.c_25d')

date={
    'title':title,
    'price':price[0].text,
    'date':date[0].text,
    'area':list(area[0].stripped_strings)
} #因为soup.select返回的是列表

#分个人和商家，每个网址有不同的辨识特点 0和1的区别
def get_link_from(who_sell=0):
     url=[]
     list_view='http://yc.58.com/pingbandiannao/0/?PGTID=0d305a36-0035-aa8a-c07f-9f7d29b50f56&ClickID=1'.format(str(who_sell))
     wb_data=requests.get(list_view)
     soup=BeautifulSoup(wb_data,'lxml')
     for link in soup.select('td.t a.t'):
         url.append(link.get('href').split('?')[0])  #以问号作为分割，取列表中前面的数据
      return url

#取得浏览量的数据，因为存放在JS中所有要在sourece中去找
def get_views_from(url):
    id=url.split('/')[-1].strip('x.shtml')
    api='http://jst1.58.com/counter?infoid={}'.format(id)  #'http://jst1.58.com/counter?infoid={}'是从source中找到的链接
    js=requests.get(api)
    views=js.text.split('=')[-1]  # 因为response中的末尾存放有浏览量
    return views

#调用函数
def get_item_info(who_sell):
     urls=get_link_from(who_sell)
     for url in urls:

        wb_data = requests.get(url)
        soup = BeautifulSoup(wb_data.text, 'lxml')
         data={
            'title':soup.title.text,
            'price':soup.select('.price')[0].text,
            'area': list(soup.select('.c_25d')[0].stripped_strings),
            'cate':'个人' if who_sell=0 else '商家', #这个判断句式要记住
            'views': get_views_from(url)
            }

get_item_info(1)
