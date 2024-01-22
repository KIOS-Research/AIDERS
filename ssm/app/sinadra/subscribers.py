import rospy
from eddi_messages.msg import BayesianNetworkOutput
import database.connection

runningSubscribers = {}

def humanInjuryCriticalityInAreaCallback(_rosMsg, _args):
    operationId = _args[0]
    for outcome in _rosMsg.output_nodes[0].outcomes:
        if outcome.outcome_name == "High":
            highOutcomePrediction = outcome.outcome_value
            database.queries.saveHumanInjuryCriticalityInAreaData(highOutcomePrediction, operationId)

def startHumanInjuryCriticalityInArea(_operationId):
    createSubscriber("eddi/sinadra/human_injury_criticality_in_area", BayesianNetworkOutput, humanInjuryCriticalityInAreaCallback, (_operationId, ))

def stopHumanInjuryCriticalityInArea():
    stopSubscriberByName("eddi/sinadra/human_injury_criticality_in_area")




###################################
############# UTILS ###############
###################################


# create a ROS subscriber with the given parameters
def createSubscriber(subscriberName, msgType, callbackFunction, args=None):
    global runningSubscribers
    if subscriberName not in runningSubscribers:
        if args is not None:
            runningSubscribers[subscriberName] = rospy.Subscriber(subscriberName, msgType, callbackFunction, args)
        else:
            runningSubscribers[subscriberName] = rospy.Subscriber(subscriberName, msgType, callbackFunction)
        print(f"Subscriber '{subscriberName}' is running.")
    print(f"Subscriber '{subscriberName}' already running.")

# stop a subscriber by name
def stopSubscriberByName(subscriberName):
    if subscriberName in runningSubscribers:
        subscriber = runningSubscribers[subscriberName]
        subscriber.unregister()
        del runningSubscribers[subscriberName] # remove the subscriber from the dictionary
        print(f"Subscriber '{subscriberName}' has been stopped.")
    else:
        print(f"No subscriber with name '{subscriberName}' is currently running.")