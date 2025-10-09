import numpy

jointAngles = [0, 0, 0, 0, 0] # theta
linkOffset = [0, 0, 0, 0, 0] # d
linkLength = [0, 0, 0, 0, 0] # a
linkTwist = [0, 0, 0, 0, 0] # alpha
matrices = []

def createMatrices():
    for i in range(5):
        arr = numpy.array([
            [numpy.cos(jointAngles[i]), (numpy.sin(jointAngles[i]) * numpy.cos(linkTwist[i]) * -1), (numpy.sin(jointAngles[i]) * numpy.sin(linkTwist[i])), (linkLength[i] * numpy.cos(jointAngles[i]))],
            [numpy.sin(jointAngles[i]), (numpy.cos(jointAngles[i]) * numpy.cos(linkTwist[i])), (numpy.cos(jointAngles[i]) * numpy.sin(linkTwist[i]) * -1), (linkLength[i] * numpy.sin(jointAngles[i]))],
            [0, numpy.sin(linkTwist[i]), numpy.cos(linkTwist[i]), linkOffset[i]],
            [0, 0, 0, 1]
        ])
        matrices.append(arr)

createMatrices()
for matrix in matrices:
    print(matrix, "\n\n")
