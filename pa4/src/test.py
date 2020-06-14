import pickle

b = [1, 2, 3]

with open('my_points.pkl', 'wb') as f:
    pickle.dump(b, f, pickle.HIGHEST_PROTOCOL)