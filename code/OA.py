#!/usr/bin/env python2.7

import numpy as np

def outcomeAssessment(samples, R_inf):
	L_samples=np.unique(np.where(samples<R_inf))
	U_samples=np.unique(np.where(samples>R_inf))
	if not L_samples.any():
		return None
	samples=list(samples)
	LPM=sum([float((R_inf - samples[x])*samples.count(samples[x]))/float(len(samples)) for x in L_samples])
	UPM=sum([float((samples[x] - R_inf)*samples.count(samples[x]))/float(len(samples)) for x in U_samples])
	xO=(float(2)/(1+np.exp(-np.log(float(UPM)/float(LPM)))))-1

	return xO

