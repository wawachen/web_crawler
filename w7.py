from bs4 import BeautifulSoup
import requests
import json
'''
url="https://www.tripadvisor.cn/Attractions-g60763-Activities-New_York_City_New_York.html#ATTRACTION_SORT_WRAPPER"
wb_data=requests.get(url)
soup=BeautifulSoup(wb_data.text,"lxml")
titles=soup.select('div.listing_title > a[target="_blank"]')
imgs=soup.select('img[width="180"]')
cates=soup.select("div.p13n_reasoning_v2")

for title,img,cate in zip(titles,imgs,cates):
    data={
        'title':title.get_text(),
        'img':img.get("src"),
        'cate':list(cate.stripped_strings)
    }
  
     print(data['title'])
print json.dumps(data, encoding="UTF-8", ensure_ascii=False)
'''
headers={
    'User-Agent':"TAUnique=%1%enc%3AM67WgPyrqRFJVTwBlG7UkmRDWnONsxCACFW%2Bz8qi1Y8%3D; TASSK=enc%3AAGm7YV3GrKIkc1cxJMQCJ%2BKcAl5z23%2BZ2sgh8WpmEOZlUeBX4TFN7Kf16Yc3cPzSxW6O5A%2FTDUcJ8KkBYasXyOm5KrQdPVouYyEYuvPUULYLHTj5kr6mWN%2FHqrtPzWLrnQ%3D%3D; __gads=ID=5bc55ce6ea8261a7:T=1499830594:S=ALNI_MYvxiFWdF-8WCUhaWs6VE548CROHQ; taMobileRV=%1%%7B%2210028%22%3A%5B60763%5D%7D; ServerPool=A; VRMCID=%1%V1*id.16631*llp.%2F-a_ttcampaign%5C.MTYpc-a_ttgroup%5C.title-m16631*e.1500813933543; _smt_uid=59659b21.22d698a5; _jzqy=1.1499871086.1500209620.1.jzqsr=baidu|jzqct=tripadvisor.-; _jzqckmp=1; CommercePopunder=SuppressAll*1500209629442; _jzqx=1.1499873208.1500258565.2.jzqsr=tripadvisor%2Ecn|jzqct=/tourism-g60763-new_york_city_new_york-vacations%2Ehtml.jzqsr=tripadvisor%2Ecn|jzqct=/attractions-g60763-activities-new_york_city_new_york%2Ehtml; TAAuth3=3%3A2af47bd1c6d87b424f86f456a659475b%3AAM0JUmPnDkeHQOz6L%2FzqQoThO2%2FJbJvq%2BHOriCWtHeVnzQ%2FRdOpoYGZsAKokgaxbpOR%2Fz65EIyURAYX%2BTOuNZIWMrZmqnGKZ%2BQcb%2FFePPQ4gS6u%2F7v%2FdJWDWmjmh5oiirMvsC170K3pV3L2lGP5OHiGcX8iYiQVaXvxfVWn75tc2xS7lGOhO5qKUNhKBeZLvjA%3D%3D; TAReturnTo=%1%%2FAttraction_Review-g60763-d105127-Reviews-Central_Park-New_York_City_New_York.html; CM=%1%PremiumMobSess%2C%2C-1%7Ct4b-pc%2C%2C-1%7CRCPers%2C%2C-1%7CWShadeSeen%2C%2C-1%7CTheForkMCCPers%2C%2C-1%7CHomeASess%2C6%2C-1%7CPremiumSURPers%2C%2C-1%7CPremiumMCSess%2C%2C-1%7CCCSess%2C%2C-1%7CCpmPopunder_2%2C1%2C-1%7CViatorMCPers%2C%2C-1%7Csesssticker%2C%2C-1%7C%24%2C%2C-1%7CPremiumORSess%2C%2C-1%7Ct4b-sc%2C%2C-1%7CMC_IB_UPSELL_IB_LOGOS2%2C%2C-1%7Cb2bmcpers%2C%2C-1%7CMC_IB_UPSELL_IB_LOGOS%2C%2C-1%7CPremMCBtmSess%2C%2C-1%7CPremiumSURSess%2C%2C-1%7CLaFourchette+Banners%2C%2C-1%7Csess_rev%2C%2C-1%7Csessamex%2C%2C-1%7CPremiumRRSess%2C%2C-1%7Cmultipers%2C%2C-1%7CTheForkORSess%2C%2C-1%7CTheForkRRSess%2C%2C-1%7Cpers_rev%2C%2C-1%7Cmds%2C1500259586055%2C1500345986%7CRBAPers%2C%2C-1%7CHomeAPers%2C%2C-1%7CPremiumMobPers%2C%2C-1%7CRCSess%2C%2C-1%7CLaFourchette+MC+Banners%2C%2C-1%7Csh%2C%2C-1%7CLastPopunderId%2C137-1859-null%2C-1%7Cpssamex%2C%2C-1%7CTheForkMCCSess%2C%2C-1%7CCCPers%2C%2C-1%7Cb2bmcsess%2C%2C-1%7CViatorMCSess%2C%2C-1%7CPremiumMCPers%2C%2C-1%7CPremiumRRPers%2C%2C-1%7CTheForkORPers%2C%2C-1%7CPremMCBtmPers%2C%2C-1%7CTheForkRRPers%2C%2C-1%7Cmultisess%2C%2C-1%7CPremiumORPers%2C%2C-1%7CRBASess%2C%2C-1%7Cperssticker%2C%2C-1%7CCPNC%2C%2C-1%7C; _gat_UA-79743238-4=1; roybatty=TNI1625!ANdPNFeeWhjgA9LgZOQkR%2FeIUOpUasGmIZUYp%2FD6WXCUEoBKA4XBdFfPNe02XK9eKcmrxT%2BVlEzXoVOJU9awxljNI6SHcbz8II3CG%2B0dKcSzeIOuoA%2FcT%2BFGL79bJU1e9Z62rg3nmm1kKba7ye9BPZibEUhwsAb3j2VkUKAKh%2FnI%2C1; _ga=GA1.2.150398951.1499831070; _gid=GA1.2.1382309640.1500209618; TASession=%1%V2ID.14622416BE3F18AE6C180A1A3C238ACF*SQ.56*MC.16631*LR.http%3A%2F%2Fbzclk%5C.baidu%5C.com%2Fadrc%5C.php%3Ftpl%3Dtpl_10085_15673_1%26l%3D1053927199%26wd%3Dtripadvisor%26issp%3D1%26f%3D8%26ie%3Dutf-8%26tn%3Dbaiduhome_pg*LP.%2F-a_ttcampaign%5C.MTYpc-a_ttgroup%5C.title-m16631*PR.39778%7C*LS.Saves*GR.54*TCPAR.32*TBR.46*EXEX.87*ABTR.5*PHTB.16*FS.37*CPU.35*HS.recommended*ES.popularity*AS.popularity*DS.5*SAS.popularity*FPS.oldFirst*TS.F367CBBA930DE5F9999ACDC34F0E0022*LF.zhCN*FA.1*DF.0*MS.-1*RMS.-1*FLO.60763*TRA.true*LD.105127; TATravelInfo=V2*AY.2017*AM.7*AD.23*DY.2017*DM.7*DD.24*A.2*MG.293916*HP.2*FL.3*RHS.47c1c_2017-07-23_1*RVL.293916_193l105125_194l60763_197l105127_197*DSM.1500259678030*RS.1; TAUD=LA-1500209133493-1*RDD-1-2017_07_16*HD-50452562-2017_07_23.2017_07_24.105127*LD-50544530-2017.7.23.2017.7.24*LG-50544532-2.1.F.; ki_t=1499831100066%3B1500258566592%3B1500260166431%3B4%3B25; ki_r=; Hm_lvt_2947ca2c006be346c7a024ce1ad9c24a=1499831073,1499871086,1499959234,1500209620; Hm_lpvt_2947ca2c006be346c7a024ce1ad9c24a=1500260166; _qzja=1.1897009113.1499831073601.1500209619750.1500258565260.1500260148501.1500260166477..0.0.30.6; _qzjb=1.1500258565260.14.0.0.0; _qzjc=1; _qzjto=14.1.0; _jzqa=1.1955327271975424000.1499831074.1500209620.1500258565.6; _jzqc=1; _jzqb=1.14.10.1500258565.1",
    'Cookie':"Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/59.0.3071.115 Safari/537.36"
}
url_save="https://www.tripadvisor.cn/Saves/776555"
wb_data=requests.get(url_save,headers=headers)
soup=BeautifulSoup(wb_data.text,'lxml')
titles=soup.select("a.title")
imgs=soup.select("a.thumbnail")
metas=soup.select("div.poi_type_tags")

for title,img,meta in zip(titles,imgs,metas):
    data={
        "title":title.get_text(),
        "imgs": img.get('src'),
        "meta":list(meta.stripped_strings)
    }
    print(data)

