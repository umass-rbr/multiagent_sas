import random


def help(attemptedAction):

    # define list of responses
    responseList = [
        'teleoperation', 
        'approval',
        'denial'
    ]
    # randomly select response, for now
    #response = random.randint(0,3)

    #return responseList[response]

    # return the proposed action and just take it
    return attemptedAction