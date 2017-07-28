from bs4 import BeautifulSoup
import requests
import time

url="https://knewone.com/discover?page=7"

#def asynweb(urlas):
 #   time.sleep(4)
wb_data = requests.get(url)
soup = BeautifulSoup(wb_data.text,'lxml')
titles = soup.select('section.content>h4>a')
imgs=soup.select('a.cover-inner>img')
links=soup.select('section.content>h4>a')

for title,img,link in zip(titles,imgs,links):
    data={
            'title':title.get('title'),
            'img':img.get('src'),
            'link':link.get('href')
        }
    print(data)

'''def asynmulti(i):
    asynweb(url+str(i))

for num in range(1,25):
    asynmulti(num)
'''