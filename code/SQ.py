#!/usr/bin/env python2.7

import numpy as np
import math
from scipy import stats

def solverQuality(mcts_samples, vi_samples):
	#compute the hellinger distance
	mu_p = np.mean(mcts_samples)
	sigma_p = np.std(mcts_samples)	
	mu_q = np.mean(vi_samples)
	sigma_q = np.std(vi_samples)

	rH = 200
	rL = -260


	h = 0
	p = []
	q = []
	p_samples = np.unique(mcts_samples,return_counts=True)
	q_samples = np.unique(vi_samples,return_counts=True)

	for x in range(0,len(p_samples[0])):
		p.append(float(p_samples[1][x])/float(len(mcts_samples)))

	for x in range(0,len(q_samples[0])):
		q.append(float(q_samples[1][x])/float(len(vi_samples)))

	all_samples = np.unique(np.concatenate((p_samples[0],q_samples[0]), axis=None))

	for i in all_samples:
		#if isin(p_samples[0],i,assume_unique=True).any():
			#get probability indeces
		p_index = np.where(p_samples[0] == i)[0]
		q_index = np.where(q_samples[0] == i)[0]

		if len(p_index) == 0:
			pi = 0
		else: 
			pi = p[p_index[0]]
		if len(q_index) == 0:
			qi = 0
		else: 
			qi = q[q_index[0]]
		h += pow(math.sqrt(pi) - math.sqrt(qi),2)
		#else:
			#pass

	h = (float(1)/math.sqrt(2))*math.sqrt(h)
	print h

	f = (float(stats.mode(mcts_samples)[0]-stats.mode(vi_samples)[0]))/float(rH - rL)
	q = np.sign(mu_p-mu_q)*pow(np.abs(f),0.1)*math.sqrt(h)

	sq = float(2)/float(1+math.exp(-q/5))

	return sq


def isin(element, test_elements, assume_unique=False, invert=False):
    "..."
    element = np.asarray(element)
    return np.in1d(element, test_elements, assume_unique=assume_unique,
                invert=invert).reshape(element.shape)