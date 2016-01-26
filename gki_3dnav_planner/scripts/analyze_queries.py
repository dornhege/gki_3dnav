#!/usr/bin/env python

import yaml
import sys
import matplotlib.pyplot as plt

def time_to_first(run_stats):
    return run_stats[0]['time']
time_to_first.min = 0
time_to_first.max = 10.5

def final_eps_reached(run_stats):
    eps = run_stats[-1]['eps']
    if eps > 10:
        eps = 10
    return eps
final_eps_reached.min = 0.9
final_eps_reached.max = 5.1

def final_cost(run_stats):
    cost = run_stats[-1]['cost']
    if cost > 100*1000:
        cost = 100*1000
    return cost
final_cost.min = 0
final_cost.max = 101*1000

def expands(run_stats):
    return sum([r['expands'] for r in run_stats])
expands.min = 0
expands.max = 3*1000

def expands_to_first(run_stats):
    return run_stats[0]['expands']
expands_to_first.min = 0
expands_to_first.max = 1000

def scatter_pairs(data, fn):
    entries = []
    first = True
    for k, v in sorted(data.iteritems()):
        for i, run in enumerate(v):
            if first:
                entries.append([fn(run)])
            else:
                entries[i].append(fn(run))
        first = False
    return entries

def make_scatter_plot(ax, data, fn):
    sp = scatter_pairs(data, fn)
    try:
        ax.set_xlim(fn.min, fn.max)
        ax.set_ylim(fn.min, fn.max)
    except AttributeError:
        pass
    ax.plot([e[0] for e in sp], [e[1] for e in sp], 'o', alpha=0.5)
    ax.plot([0,100*1000], [0, 100*1000], '-', color='k', linewidth=2)
    labels = sorted(data.keys())
    ax.set_xlabel(labels[0])
    ax.set_ylabel(labels[1])
    ax.set_title(fn.__name__)

def make_time_histogram(ax, data, fn):
    try:
        ax.set_xlim(fn.min, fn.max)
        ax.set_ylim(0.0, 1.05)
    except AttributeError:
        pass
    data_time_points = {}
    for k, v in data.iteritems():
        for run in v:
            time_point = fn(run)
            if not k in data_time_points:
                data_time_points[k] = [time_point]
            else:
                data_time_points[k].append(time_point)
    for k, v in data_time_points.iteritems():
        v.sort()
    def make_hist(times):
        sum_acc = 0.0
        hist = {}
        hist[0.0] = 0.0
        for t in times:
            sum_acc += 1.0
            hist[t] = sum_acc
        hist = dict((k, v/sum_acc) for k,v in hist.iteritems())
        return hist
    for k in sorted(data_time_points.keys()):
        h = make_hist(data_time_points[k])
        points = [(k, v) for k,v in sorted(h.iteritems())]
        ax.plot([e[0] for e in points], [e[1] for e in points], '-')
    ax.plot([0, 10], [0.95, 0.95], '-', color='k')
    ax.legend(sorted(data_time_points.keys()), 'lower right')
    ax.set_title(fn.__name__)

def main():
    pq_name = sys.argv[1]
    param_name = sys.argv[2]
    
    with open(pq_name, 'r') as f:
        data = yaml.load(f)
    assert len(data[param_name+'_true_stats']) == len(data[param_name+'_false_stats'])

    f, axx = plt.subplots(3, 2)

    make_scatter_plot(axx[0, 0], data, time_to_first)
    make_scatter_plot(axx[0, 1], data, final_eps_reached)
    make_scatter_plot(axx[1, 0], data, final_cost)
    make_time_histogram(axx[1, 1], data, time_to_first)
    make_scatter_plot(axx[2, 0], data, expands_to_first)
    make_scatter_plot(axx[2, 1], data, expands)

    plt.show()

if __name__=="__main__":
    main()

