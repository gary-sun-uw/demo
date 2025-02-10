import pandas as pd
from sklearn.linear_model import LogisticRegression
from sklearn.tree import DecisionTreeClassifier
from sklearn.tree import export_text
from sklearn.model_selection import train_test_split
import numpy as np

device_data=pd.read_csv('../data/gary_data_2.csv', index_col=0)
device_data['state']=pd.Series(dtype=str)
device_data.loc[:100, 'state']='s'
device_data.loc[100:200, 'state']='t'
device_data.loc[200:300, 'state']='r'

X=device_data.drop(['state'],axis=1)
y=device_data['state']

X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2)
print(y_train.value_counts(), y_test.value_counts())

tree_model=DecisionTreeClassifier(max_depth=3)
tree_model.fit(X_train,y_train)
score=tree_model.score(X_test,y_test)
print('Score:', score)

print('Fully fit tree:')
tree_model.fit(X,y)
r = export_text(tree_model, feature_names=list(X.columns))
print(r)