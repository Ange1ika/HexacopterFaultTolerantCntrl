from plotter import load_csv
from plotter import Plotter

if __name__ == "__main__":
    log = load_csv("simulation_log.csv")
    plotter = Plotter(log)
    plotter.plot_all()