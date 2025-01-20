import pandas as pd
from sklearn.linear_model import LogisticRegression
from sklearn.tree import DecisionTreeClassifier
from sklearn.model_selection import train_test_split
import numpy as np

label_data=pd.read_csv('data.csv')
device_data=pd.read_csv('../data/gary_data_1.csv')

merged_data=pd.concat([label_data,device_data],axis=1)
merged_data=merged_data.dropna()
merged_data.to_csv('merged_data.csv',index=False)

X=merged_data.drop(['state','time'],axis=1)
y=merged_data['state']
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)
model=LogisticRegression()
model.fit(X_train,y_train)
score=model.score(X_test,y_test)
print('score', score)
print(model.coef_.shape)

tree_model=DecisionTreeClassifier(max_depth=3)
tree_model.fit(X_train,y_train)
score=tree_model.score(X_test,y_test)
print('score', score)

# Print a ascii tree
from sklearn.tree import export_text
r = export_text(tree_model, feature_names=list(X.columns))
print(r)

