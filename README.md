# Open-source C++ implementation of Slack Induction by String Removals for vehicle routing and team orienteering problems

### Authors: Martin Pajerský, Václav Sobotka, Hana Rudová, Faculty of Informatics, Masaryk University, Brno, Czech Republic

The SISR algorithm for vehicle routing problems is implemented based on [1].
Authors of this implementation proposed its extension for team orienteering problems (an accompanying publication TBA).

## Features of the implementation
* Vehicle routing: achieved higher speedups than the original solver [1], up to 10 times on the largest instances
* Team orienteering: computing 49 new best-known solutions and delivering about 85% of the best-known solutions across all considered variants of orienteering problems

## Repository

The repository contains a collection of C++ implementations of the Slack
Induction by String Removals (SISR) algorithm for the following problems:

* Capacitated vehicle routing problem (branch __cvrp__)
* Vehicle routing problem with time windows (branch __vrptw__)
* Pickup and delivery problem with time windows (branch __pdptw__)
* Team orienteering problem (branch __top__)
* Team orienteering problem with time windows (branch __toptw__)
* Capacitated team orienteering problem (branch __ctop__)

The individual branches contain further information, including benchmark data
compatible with the solver and instructions for building and running the solver.

## Sources

[1] Jan Christiaens, Greet Vanden Berghe (2020)
Slack Induction by String Removals for Vehicle Routing Problems.
Transportation Science 54(2):417-433. https://doi.org/10.1287/trsc.2019.0914
