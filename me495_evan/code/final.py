import modern_robotics as mr
import numpy as np
import csv
import matplotlib.pyplot as plt

# inital delta t
dt = 0.01
# ty is a transformation matrix. it rotates y by 90 deg
Ty = np.array([[0, 0, 1, 0],
               [0, 1, 0, 0],
               [-1, 0, 0, 0],
               [0, 0, 0, 1]])
# end effector init
Tsei = np.array([[0, 0, 1, 0],
                 [0, 1, 0, 0],
                 [-1, 0, 0, 0.5],
                 [0, 0, 0, 1]])
# cube init
Tsci = np.array([[1, 0, 0, 1],
                 [0, 1, 0, 1],
                 [0, 0, 1, 0.025],
                 [0, 0, 0, 1]])
# init stand off  need to postmultiply
Tis = np.dot(Tsci, Ty) + np.array([[0, 0, 0, 0],
                                   [0, 0, 0, 0],
                                   [0, 0, 0, 0.2],
                                   [0, 0, 0, 0]])

# cube goal
Tscf = np.array([[0, 1, 0, 1],
                 [-1, 0, 0, -1],
                 [0, 0, 1, 0.025],
                 [0, 0, 0, 1]])

# final stand off
Tfs = np.dot(Tscf, Ty) + np.array([[0, 0, 0, 0],
                                   [0, 0, 0, 0],
                                   [0, 0, 0, 0.2],
                                   [0, 0, 0, 0]])

# end effect grasp
Tceg = np.array([[1, 0, 0, 0.01],
                 [0, 1, 0, 0.01],
                 [0, 0, 1, 0.01],
                 [0, 0, 0, 1]])

Tsgi = np.dot(Tsci, Tceg)
Tsgi = np.dot(Tsgi, Ty)
Tsgf = np.dot(Tscf, Tceg)
Tsgf = np.dot(Tsgf, Ty)

k = 1  # numbers of generations, the more the more accurate
Tf = 5  # time period for the movement
N = 5 * k / 0.01
method = 3  # qubic

#generate trajectory

def TrajectoryGenerator():
    align1 = mr.ScrewTrajectory(Tsei, Tis, Tf, N, method)
    align2 = mr.ScrewTrajectory(Tis, Tsgi, Tf, N, method)
    align3 = mr.ScrewTrajectory(Tsgi, Tis, Tf, N, method)
    align4 = mr.ScrewTrajectory(Tis, Tfs, Tf, N, method)
    align5 = mr.ScrewTrajectory(Tfs, Tsgf, Tf, N, method)
    align6 = mr.ScrewTrajectory(Tsgf, Tfs, Tf, N, method)

    row = len(align1)

    output = np.array([[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])

    for i in range(row):
        R, p = mr.TransToRp(align1[i])

        output1 = np.array(
            [[R[0][0], R[0][1], R[0][2], R[1][0], R[1][1], R[1][2], R[2][0], R[2][1], R[2][2], p[0], p[1], p[2], 0]])

        output = np.concatenate((output, output1), axis=0)

    row = len(align2)

    for i in range(row):
        R, p = mr.TransToRp(align2[i])

        output1 = np.array([[R[0][0], R[0][1], R[0][2], R[1][0], R[1][1], R[1][2], R[2][0], R[2][1],
                             R[2][2], p[0], p[1], p[2], 0]])
        output = np.concatenate((output, output1), axis=0)
    # close grip

    for i in range(100):
        grip = np.array([[R[0][0], R[0][1], R[0][2], R[1][0], R[1][1], R[1][2], R[2][0],
                          R[2][1], R[2][2], p[0], p[1], p[2], 1]])
        output = np.concatenate((output, grip), axis=0)

    row = len(align3)

    for i in range(row):
        R, p = mr.TransToRp(align3[i])

        output1 = np.array([[R[0][0], R[0][1], R[0][2], R[1][0], R[1][1], R[1][2],
                             R[2][0], R[2][1], R[2][2], p[0], p[1], p[2], 1]])
        output = np.concatenate((output, output1), axis=0)

    row = len(align4)

    for i in range(row):
        R, p = mr.TransToRp(align4[i])

        output1 = np.array([[R[0][0], R[0][1], R[0][2], R[1][0], R[1][1], R[1][2],
                             R[2][0], R[2][1], R[2][2], p[0], p[1], p[2], 1]])
        output = np.concatenate((output, output1), axis=0)

    row = len(align5)

    for i in range(row):
        R, p = mr.TransToRp(align5[i])

        output1 = np.array([[R[0][0], R[0][1], R[0][2], R[1][0], R[1][1],
                             R[1][2], R[2][0], R[2][1], R[2][2], p[0], p[1], p[2], 1]])
        output = np.concatenate((output, output1), axis=0)
    # open grip

    for i in range(100):
        grip = np.array([[R[0][0], R[0][1], R[0][2], R[1][0], R[1][1], R[1][2],
                          R[2][0], R[2][1], R[2][2], p[0], p[1], p[2], 0]])
        output = np.concatenate((output, grip), axis=0)

    row = len(align6)

    for i in range(row):
        R, p = mr.TransToRp(align6[i])

        output1 = np.array([[R[0][0], R[0][1], R[0][2], R[1][0], R[1][1],
                             R[1][2], R[2][0], R[2][1], R[2][2], p[0], p[1], p[2], 0]])
        output = np.concatenate((output, output1), axis=0)
    return output

#generate next config
def NextState(config, control, s, dt):
    for n, i in enumerate(control):

        if abs(i) > s:

            if i >= 0:
                control[n] = s
            else:
                control[n] = -s

    theta = np.array([[control[0] * dt], [control[1] * dt], [control[2] * dt], [control[3] * dt]])
    # get twist
    V = np.dot(F, theta)

    wbz = V[0][0]

    vx = V[1][0]
    vy = V[2][0]

    if wbz == 0:
        qb = np.array([[0], [vx], [vy]])

    else:
        qb = np.array([[wbz], [(vx * np.sin(wbz) + vy * (np.cos(wbz) - 1)) / wbz], [(vy * np.sin(wbz) + vx * (1 - np.cos(wbz))) / wbz]])

    fi = config[0]
    A = np.array([[1, 0, 0], [0, np.cos(fi), -np.sin(fi)], [0, np.sin(fi), np.cos(fi)]])
    dq = np.dot(A, qb)

    q1 = config[0] + dq[0][0]
    q2 = config[1] + dq[1][0]
    q3 = config[2] + dq[2][0]

    a1 = config[3] + control[4] * dt
    a2 = config[4] + control[5] * dt
    a3 = config[5] + control[6] * dt
    a4 = config[6] + control[7] * dt
    a5 = config[7] + control[8] * dt
    w1 = config[8] + control[0] * dt
    w2 = config[9] + control[1] * dt
    w3 = config[10] + control[2] * dt
    w4 = config[11] + control[3] * dt

    config = np.array([q1, q2, q3, a1, a2, a3, a4, a5, w1, w2, w3, w4, 0])

    return config

#gives speed control directly
def FeedbackControl(Xd, Xdn, X, Kp, Ki, dt, Xerr):
    Xer = mr.MatrixLog6(np.dot(mr.TransInv(X), Xd))
    Xer = mr.se3ToVec(Xer)
    Vd = (1 / dt) * mr.MatrixLog6(np.dot(mr.TransInv(Xd), Xdn))
    Vd = mr.se3ToVec(Vd)
    adjoint = mr.Adjoint(np.dot(mr.TransInv(X), Xd))
    Xerr = Xerr + Xer * dt

    V = np.dot(adjoint, Vd) + np.dot(Kp, Xer) + np.dot(Ki, Xerr)
    Je = np.concatenate((Jba, Jb), axis=1)

    Js = np.linalg.pinv(Je)
    Vt = np.dot(Js, V)


    return (Vt,Xer)


if __name__ == '__main__':
    #inital condition
    s = 30
    l = 0.47 / 2
    w = 0.15
    r = 0.0475

    F = (r / 4.0) * np.array([[-1 / (l + w), 1 / (l + w), 1 / (l + w), -1 / (l + w)],
                            [1, 1, 1, 1],
                            [-1, 1, -1, 1]])

    dt = 0.01
    ref = TrajectoryGenerator()
    row2 = len(ref)

    Kp = 1*np.identity(6)
    Ki = 0*np.identity(6)
    config = np.array([0.5, -0.417, 0.5, 0, 0, 0, -1.57, 0, 0, 0, 0, 0, 0])

    F6 = np.array([[0, 0, 0, 0],
                   [0, 0, 0, 0],
                   [F[0][0], F[0][1], F[0][2], F[0][3]],
                   [F[1][0], F[1][1], F[1][2], F[1][3]],
                   [F[2][0], F[2][1], F[2][2], F[2][3]],
                   [0, 0, 0, 0]])

    M = np.array([[1, 0, 0, 0.033], [0, 1, 0, 0], [0, 0, 1, 0.6546], [0, 0, 0, 1]])
    Blist = np.array([[0, 0, 1, 0, 0.033, 0], [0, -1, 0, -0.5076, 0, 0], [0, -1, 0, -0.3526, 0, 0],
                      [0, -1, 0, -0.2176, 0, 0], [0, 0, 1, 0, 0, 0]]).T

    Tb0 = np.array([[1, 0, 0, 0.1662], [0, 1, 0, 0], [0, 0, 1, 0.0026], [0, 0, 0, 1]])
    Xerr=0

    #calcutate output write it into csv
    with open('Best_Xer.csv', 'w', newline='') as f:
        thewriter = csv.writer(f)

        for i in range(row2-2):
            i += 1

            R = np.array([[ref[i][0], ref[i][1], ref[i][2]], [ref[i][3], ref[i][4], ref[i][5]],
                          [ref[i][6], ref[i][7], ref[i][8]]])
            p = np.array([ref[i][9], ref[i][10], ref[i][11]])

            Xd = mr.RpToTrans(R, p)
            i += 1

            R1 = np.array([[ref[i][0], ref[i][1], ref[i][2]], [ref[i][3], ref[i][4], ref[i][5]],
                           [ref[i][6], ref[i][7], ref[i][8]]])
            p1 = np.array([ref[i][9], ref[i][10], ref[i][11]])

            Xdn = mr.RpToTrans(R1, p1)

            fi = config[0]
            x = config[1]
            y = config[2]
            Tsb = np.array([[np.cos(fi), -np.sin(fi), 0, x], [np.sin(fi), np.cos(fi), 0, y],
                            [0, 0, 1, 0.0963], [0, 0, 0, 1]])
            angle = np.array([config[3], config[4], config[5], config[6], config[7]])
            T0e = mr.FKinBody(M, Blist, angle)
            X = np.dot(Tsb, np.dot(Tb0, T0e))

            iTb0 = mr.TransInv(Tb0)

            iT0e = mr.TransInv(T0e)

            Jb = mr.JacobianBody(Blist, angle)
            Jba = np.dot(mr.Adjoint(np.dot(iT0e, iTb0)), F6)


            (control, Xe) = FeedbackControl(Xd, Xdn, X, Kp, Ki, dt, Xerr)

            if i==2:

                nXerr = np.array([[Xe[0]], [Xe[1]], [Xe[2]], [Xe[3]], [Xe[4]], [Xe[5]]])

            Xerr = Xe

            Xe=np.array([Xe])




            nXerr=np.concatenate((nXerr, Xe.T), axis=1)



            config = NextState(config, control, s, dt)

            config[12]= ref[i][12]
            output = (config)
            thewriter.writerow(output)
















        plt.plot(nXerr[0], label='wx')
        plt.plot(nXerr[1],label='wy')
        plt.plot(nXerr[2],label='wz')
        plt.plot(nXerr[3],label='x')
        plt.plot(nXerr[4],label='y')
        plt.plot(nXerr[5],label='z')
        plt.xscale('log')

        plt.legend()


        plt.show()
