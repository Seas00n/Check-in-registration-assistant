import pandas as pd
from pandas import DataFrame
from xpinyin import Pinyin
file_path = "qiandao.xlsx"

df = pd.read_excel(file_path,sheet_name="Sheet")
name_list = []

p = Pinyin()
def get_pinyin(str):
    name = str.split(",")
    name = name[0]
    py = p.get_pinyin(name)
    n_list = py.split('-')
    pystr = ""
    for i in range(len(n_list)):
        pystr+=n_list[i][0]
    return pystr

for name in df["Student Name"]:
    name_list.append(get_pinyin(name))


column_title = input("行标题：")
has_column = False
for column in df.columns:
    if column == column_title:
        has_column = True
        break
if not has_column:
    df[column_title] = None
    DataFrame(df).to_excel(file_path, sheet_name="Sheet",index=False, header=True)

print("Enter Q to break")

while True:
    name = input("小写缩写：")
    if name == "Q":
        break
    
    name_same = []
    for idx in range(len(name_list)):
        if name_list[idx]==name:
            name_same.append(idx)


    if len(name_same)==0:
        print("无学生")
        continue
    elif len(name_same)>1:
        print("多个学生")
        for i in name_same:
            print("[{}]:{}".format(i,df["Student Name"][i]))
        idx = input("哪一个学生：")
        while True:
            if idx=="Q":
                break
            elif not idx.isdigit():
                idx = input("哪一个学生：")
                continue
            elif idx.isdigit():
                idx = int(idx)
                if idx in set(name_same):
                    name_same = [idx]
                    break
                else:
                    idx = input("哪一个学生：")
                    continue

        if idx == "Q":
            break
    
    idx = name_same[0]
    print(df["Student Name"][idx])
    df[column_title][idx] = 1
    DataFrame(df).to_excel(file_path, sheet_name="Sheet",index=False, header=True)
    continue