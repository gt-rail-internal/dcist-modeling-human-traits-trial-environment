import itertools
import random
from munkres import Munkres

# generate a matrix of the impact of each of T traits on each of J jobs
#   input: J (number of jobs), T (number of traits)
#   output: I (JxT matrix of the impact of each trait) 
def generateImpactMatrix(J, T):
    I = []
    # for each job
    for j in range(J):
        impacts = []
        # for each trait
        for t in range(T):
            # generate the impact (assuming normal distribution)
            impact_val = random.gauss(0, .1)
            impacts.append(impact_val)
        I.append(impacts)

    # return the impact matrix
    return I


# generate a matrix of N humans with T traits
#   input: N (number of humans), T (number of traits)
#   output: humans (NxT matrix of human traits)
def generateHumans(N, T):
    humans = []
    # for each human
    for n in range(N):
        traits = []
        # for each trait
        for t in range(T):
            # generate the trait (assuming normal distribution)
            trait_val = random.gauss(.5, .2)
            trait_val = 0 if trait_val < 0 else 1 if trait_val > 1 else trait_val
            traits.append(trait_val)
        humans.append(traits)
    
    # return the matrix of trait values
    return humans


# pulls every human subset
#   input: N (number of rows), J (number of assignments))
#   output: subsets (N^(J-1)xJ)
def pullSubsets(N, J, subsets=[], subset=[], slot=-1, all=False):
    subsets = [list(x) for x in itertools.combinations(range(N), J)]
    return subsets

    # if still have more slot to subset, fill children
    if slot < J - 1:
        # add each worker to the subset
        for worker in range(N):
            # only add if worker is not already in subset
            if worker not in subset:
                new_subset = sorted([x for x in subset] + [worker])
                pullSubsets(N, J, subsets, new_subset, slot + 1)
        
    # otherwise, this is the last slot, check if subset exists and return
    elif subset not in subsets:
        subsets.append(subset)

    # top level return
    return subsets    


# generates row/column combinations
#   input: N (number of rows, number of columns)
#   output: combinations (list of all combinations of length 1:N)
def generateCombinations(N):
    combinations = []    
    item_list = [x + 1 for x in range(N)] + [-(x + 1) for x in range(N)]
    for r in range(N+1)[1:]:
        combinations += [list(x) for x in itertools.combinations(item_list, r)]
    return combinations


# generate a matrix of the predicted performance of each human on each job
#   input: humans (number of humans), I (impact matrix)
#   output: P (NxJ matrix of the human performance in each job)
def generateHumanPredictedPerformance(humans, I):
    P = []
    # for each human
    for human in humans:
        performances = []
        for j in range(J):
            # element-wise multiply the human traits and the impact vector
            performance = [human[i] * I[j][i] for i in range(len(I[j]))]
            # add to the human's performances
            performances.append(sum(performance))
        P.append(performances)
    
    # return the predicted performances
    return P


# generate a matrix of the live performance of each human on each job
#   input: humans (number of humans), I (impact matrix)
#   output: P (NxJ matrix of the human performance in each job)
def generateHumanLivePerformance(humans, I):
    P = []
    # for each human
    for human in humans:
        performances = []
        for j in range(J):
            # element-wise multiply the human traits and the impact vector
            performance = [human[i] * I[j][i] for i in range(len(I[j]))]
            # add or remove up to 10%
            performance = [.9 * x + random.random() * .2 * x for x in performance]
            # gaussian noise of 10% SD
            performance = [x + random.gauss(0, .1 * x) for x in performance]
            # add to the human's performances
            performances.append(sum(performance))
        P.append(performances)
    
    # return the live performances
    return P


# optimally assigns each human worker to each job 
#   input: S (NxJ matrix of human job cost)
#   output: bool (whether a solution was found), a (job assignment of each worker)
def hungarian(S):
    # relies on the Munkres library (via Pip)
    m = Munkres()
    indexes = m.compute(S)
    a = [col for row, col in indexes]
    return a


# trial the experiment
print("Starting trial run")

# step 0: set parameters
J = 3  # number of jobs
N = 30  # number of workers
T = 5  # number of traits

# step 1: generate impact matrix
print("  Generating impact matrix")
I = generateImpactMatrix(J, T)

# step 2: collect human data, traits and live performance
print("  Generating humans")
humans = generateHumans(N, T)
print("  Generating human live performances")
live_S = generateHumanLivePerformance(humans, I)

# step 3: predict human performance
print("  Generating human predicted performances")
predicted_S = generateHumanPredictedPerformance(humans, I)

# step 4: take subsets of humans
print("  Pulling subsets of humans to test")
subsets = pullSubsets(len(live_S), len(live_S[0]))

# step 4: for each subset, compare predicted assignment vs actual assignment
print("  Comparing human assignments")
correct_assignments = 0  # number of correct assignments
exact_assignments = 0  # number of exact assignments

for s in subsets:
    # pull predicted and live humans for this subset
    pred = [predicted_S[x] for x in s]
    live = [live_S[x] for x in s]

    #print("dim", len(live), "x", len(live[0]))
    #print(">", s)
    
    # determine their optimal assignments
    pred_a = hungarian(pred)
    live_a = hungarian(live)
    pred_a = pred_a
    live_a = live_a

    # add to analysis
    num_match = sum([1 for i in range(len(pred_a)) if pred_a[i] == live_a[i]])
    correct_assignments += num_match
    if num_match == len(pred_a):
        exact_assignments += num_match

print("")
print("RESULTS:")
print("Total assignments", len(subsets))
print("Correct assignments", round(100 * correct_assignments / len(subsets) / len(subsets[0]), 2))
print("Exact assignments", round(100 * exact_assignments / len(subsets) / len(subsets[0]), 2))
