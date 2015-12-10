from sklearn import svm
from sklearn.grid_search import GridSearchCV


def tune_svc(X, y):
    tuned_parameters = [{'kernel': ['rbf', 'linear'], 'C': [0.01, 0.1, 1, 10, 100]}]

    grid = GridSearchCV(svm.SVC(), tuned_parameters, cv=5)
    grid.fit(X, y)

    return grid


def print_grid_search_report(grid):
    print("All Parameters Searched:")
    for params, mean_score, scores in grid.grid_scores_:
        print("%0.3f (+/-%0.03f) for %r" % (mean_score, scores.std() * 2, params))
    print(" ")

    print("Optimal Parameters:")
    print (grid.best_params_)

    print("Best Score:")
    print(grid.best_score_)
