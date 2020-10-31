#!/usr/bin/env python2.7

import numpy as np
import math


def solverQuality(mcts_samples, vi_samples):
	#compute the hellinger distance
	mu_p = np.mean(mcts_samples)
	sigma_p = np.std(mcts_samples)	
	mu_q = np.mean(vi_samples)
	sigma_q = np.std(vi_samples)

	h = 1 - math.sqrt((2*sigma_p*sigma_q)/(pow(sigma_p,2)+pow(sigma_q,2)))*math.exp(-0.25*pow(mu_p-mu_q,2)/(pow(sigma_p,2)+pow(sigma_q,2)))	

	q = pow(np.abs(mu_p-mu_q),0.5)*math.sqrt(h)

	#print h, q
	sq = 2/(1+math.exp(-q/5))

	return sq

