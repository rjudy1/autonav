#!/usr/bin/env python

# ********************************************* #
# Cedarville University                         #
# AutoNav Senior Design Team 2020-2021          #
# Performance Analyzer Class                    #
# ********************************************* #

import datetime

class PerformanceAnalyzer():

    def __init__(self, label):
        self.label = label
        self.start_time = 0
        self.runtime = 0
        self.rolling = [0] * 10
        self.rolling_indx = 0

    def start(self):
        self.start_time = datetime.datetime.now()

    def stop(self):
        self.runtime = float((datetime.datetime.now() - self.start_time).microseconds) / float(1000)
        self.rolling[self.rolling_indx] = (self.runtime)
        self.rolling_indx = (self.rolling_indx + 1) % 10
        return self.runtime

    def avg_time(self):
        return float(sum(self.rolling)/len(self.rolling))

    def print_res(self):
        print("{} | Runtime: {} ms | Average Runime: {} ms".format(self.label, self.runtime, self.avg_time()))


class PerformanceAnalyzerLite(PerformanceAnalyzer):

    def stop(self):
        PerformanceAnalyzer.stop(self)
        self.print_res()
