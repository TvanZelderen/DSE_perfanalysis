import numpy as np
import random


print()
print()

def random_weights(no_of_criteria): # create random weights
    doesnt_sum = True
    while doesnt_sum == True:
        trial = np.random.dirichlet(np.ones(no_of_criteria)) # random numbers, all positive, sum of them is 1
        trial = np.round(trial, 3) # round, 3 decimal places
        if np.sum(trial) == 1:
            doesnt_sum = False
            return trial # np.ndarray of new, random weights


def iterator(lim, grades, no_of_criteria): # lim = number of iterations

    # drone, spacex, paraglider, wings
    tot_scores = [0,0,0,0]
    list_of_losers = [0,0,0,0]
    prev_weights = []

    # loop
    a = 0
    while a != lim:
        new_weights = random_weights(no_of_criteria)

        # check sum == 1
        if np.sum(new_weights) == 1:
            if new_weights not in prev_weights: # ensure that current weights havent been used yet
                scores = grades @ new_weights
                tot_scores[np.argmax(scores)] += 1
                list_of_losers[np.argmin(scores)] += 1

                winners = [score * 100 / lim for score in tot_scores]
                losers = [score * 100/ lim for score in list_of_losers]

                a += 1
                # print(scores)
    return winners, losers



concepts = ["drone", "spacex", "paraglider", "wings"]
criteria = ["mass", "landing accuracy", "reliability", "development risk", "reusability", "sustainability"]
grades = np.array([
[3,5,3,4,3,3], # drone
[0,2,4,1,2,1], # spacex
[5,3,2,2,4,5], # paraglider
[3,3,4,4,5,5]  # wings
])

og_weights = np.array([0.25, 0.10, 0.20, 0.15, 0.15, 0.15])


# randomizing weights
print("Iterating with all criteria, randomized weights")
print(iterator(10000, grades, 6)) # sensitivity of the criteria weighing
print()
print()





# removing each criteria and seeing how it affects the scores
print("Remove a criterion and see how it affects overall score")
def remove_criteria(grades, og_weights, crit_index): # crit index from 0 to 5
    new_grades = np.delete(grades, crit_index, 1) # 1 means column
    new_weights = np.delete(og_weights, crit_index)

    return new_grades, new_weights

list_of_winners = [0,0,0,0]
list_of_losers = [0,0,0,0]
for i in range(0, 5):
    
    new_grades = remove_criteria(grades, og_weights, i)[0]
    new_weights = remove_criteria(grades, og_weights, i)[1]

    scores = new_grades @ new_weights 

    if np.argmax(scores) != 3: print("removed ", criteria[i], ", ", concepts[np.argmax(scores)], " won")
    list_of_winners[np.argmax(scores)] += 1
    list_of_losers[np.argmin(scores)] += 1

    print("removed ", criteria[i], ". results of randomizing weights:")
    print(iterator(10000, new_grades, 5))
    print()
    

print()
print()


# changing the grades
def grade_changer(grades, no_of_changes): # grades is a 2d array, no of changes is an int
    no_of_grades = len(grades[0]) * len(grades)
    indices_to_change = random.sample(range(no_of_grades), no_of_changes)
    flat_grades = grades.ravel()
    for index in indices_to_change:
        if flat_grades[index] == 0:
            flat_grades[index] += 1
        elif flat_grades[index] == 5:
            flat_grades[index] -= 1
        else:
            add = random.sample([True, False], 1)
            if add == True:
                flat_grades[index] += 1
            else:
                flat_grades[index] -= 1

    new_grades = flat_grades.reshape(4, len(grades[0]))

    return new_grades

changed_grades_iterations = 10000
no_of_wins = [0,0,0,0]
for i in range(0,6):
    j = 0
    while j != changed_grades_iterations:
        scores = grade_changer(grades, i) @ og_weights
        no_of_wins[np.argmax(scores)] += 1
        j += 1
    for k in range(len(no_of_wins)): no_of_wins[k] = round(no_of_wins[k] * 100/changed_grades_iterations)
    print(no_of_wins, i, " changes")
    no_of_wins = [0,0,0,0]
