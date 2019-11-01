import pickle
import matplotlib.pyplot as plt

if __name__ == "__main__":
    runname = 'test-0'
    plt.plot(pickle.load(
        open("learning_curves/{}_rewards.pkl".format(runname), 'rb')))
    plt.title("Total Rewards")
    plt.show()
    agent_rewards = pickle.load(
        open("learning_curves/{}_agrewards.pkl".format(runname), 'rb'))
    for i, agent_reward in enumerate(agent_rewards):
        plt.plot(agent_reward, label="Agent {}".format(i))
    plt.title("Agent Rewards")
    plt.show()
