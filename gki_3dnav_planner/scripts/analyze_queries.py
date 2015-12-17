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
    ax.plot([e[0] for e in sp], [e[1] for e in sp], 'o')
    ax.plot([0,100*1000], [0, 100*1000], '-', color='k', linewidth=2)
    labels = sorted(data.keys())
    ax.set_xlabel(labels[0])
    ax.set_ylabel(labels[1])
    ax.set_title(fn.__name__)


def main():
    pq_name = sys.argv[1]
    with open(pq_name, 'r') as f:
        data = yaml.load(f)
    assert len(data['freespace_stats']) == len(data['no_freespace_stats'])

    f, axx = plt.subplots(3, 2)

    make_scatter_plot(axx[0, 0], data, time_to_first)
    make_scatter_plot(axx[0, 1], data, final_eps_reached)
    make_scatter_plot(axx[1, 0], data, final_cost)
    make_scatter_plot(axx[2, 0], data, expands_to_first)
    make_scatter_plot(axx[2, 1], data, expands)

    ##sp = scatter_pairs(data, time_to_first)
    #sp = scatter_pairs(data, final_cost)
    #print sp
    ##plt.xlim(0, 10.5)
    ##plt.ylim(0, 10.5)
    #plt.plot([e[0] for e in sp], [e[1] for e in sp], 'o')
    #plt.plot([0,100*1000], [0, 100*1000], '-', color='k', linewidth=2)
    #labels = sorted(data.keys())
    #plt.xlabel(labels[0])
    #plt.ylabel(labels[1])
    plt.show()
    #print data

if __name__=="__main__":
    main()

