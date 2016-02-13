# -*- coding: utf-8 -*-
"""
Created on Sat Feb 13 12:43:40 2016

@author: chao
"""
from scpye.data_reader import DataReader

base_dir = '/home/chao/Workspace/bag'
color = 'red'
mode = 'slow_flash'
side = 'north'
bag_ind = 1
min_area = 10

dr = DataReader(base_dir, color=color, mode=mode, side=side)
c = dr.load_count(bag_ind)