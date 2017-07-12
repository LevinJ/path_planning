#!/usr/bin/env python

from sklearn.naive_bayes import GaussianNB
import json
import numpy as np


def main():
	gnb = GaussianNB()
	with open('train.json', 'r') as f:
		j = json.load(f)
	print(j.keys())
	X = j['states']
	X = np.array(X)
	
# 	X[:,1] = X[:,1] % 4
	Y = j['labels']
# 	Y = np.array(Y).reshape(-1,1)
	gnb.fit(X, Y)
	
	fraction_correct = gnb.score(X, Y)
	print(Y)

	print("training, You got {} percent correct".format(100 * fraction_correct))

	with open('test.json', 'r') as f:
		j = json.load(f)

	X = j['states']
	X = np.array(X)
# 	X[:,1] = X[:,1] % 4
	Y = j['labels']
	
	fraction_correct = gnb.score(X, Y)
	print(Y)

	print("testing, You got {} percent correct".format(100 * fraction_correct))

if __name__ == "__main__":
	main()