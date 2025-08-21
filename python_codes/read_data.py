import pandas as pd
import matplotlib.pyplot as plt

data=pd.read_csv("data.csv")

#print(data.head())
fig1, (ax1,ax2,ax3) = plt.subplots(1,3,figsize = (12,10))
ax1.plot(data["time"],data["Vx_desired"],marker='o',linestyle='-',color='b')
ax1.set_title("Vx_desired")
ax1.set_xlabel("time")
ax1.set_ylabel("Vx_desired")
ax1.grid(True)

ax2.plot(data["time"],data["Vx"],marker='o',linestyle='-',color='b')
ax2.set_title("Vx")
ax2.set_xlabel("time")
ax2.set_ylabel("Vx")
ax2.grid(True)

ax3.plot(data["time"],data["e1"],marker='o',linestyle='-',color='b')
ax3.set_title("e1")
ax3.set_xlabel("time")
ax3.set_ylabel("e1")
ax3.grid(True)

fig2, (ax1,ax2) = plt.subplots(1,2,figsize = (12,10))
ax1.plot(data["time"],data["Vy_desired"],marker='o',linestyle='-',color='b')
ax1.set_title("Vy_desired")
ax1.set_xlabel("time")
ax1.set_ylabel("Vy_desired")
ax1.grid(True)

ax2.plot(data["time"],data["Vy"],marker='o',linestyle='-',color='b')
ax2.set_title("Vy")
ax2.set_xlabel("time")
ax2.set_ylabel("Vy")
ax2.grid(True)

fig3, (ax1,ax2,ax3) = plt.subplots(1,3,figsize = (12,10))
ax1.plot(data["time"],data["Ax_desired"],marker='o',linestyle='-',color='b')
ax1.set_title("Ax_desired")
ax1.set_xlabel("time")
ax1.set_ylabel("Ax_desired")
ax1.grid(True)

ax2.plot(data["time"],data["Ax"],marker='o',linestyle='-',color='b')
ax2.set_title("Ax")
ax2.set_xlabel("time")
ax2.set_ylabel("Ax")
ax2.grid(True)

ax3.plot(data["time"],data["e2"],marker='o',linestyle='-',color='b')
ax3.set_title("e2")
ax3.set_xlabel("time")
ax3.set_ylabel("e2")
ax3.grid(True)

fig4, (ax1,ax2) = plt.subplots(1,2,figsize = (12,10))
ax1.plot(data["time"],data["psi_desierd"],marker='o',linestyle='-',color='b')
ax1.set_title("psi_desierd")
ax1.set_xlabel("time")
ax1.set_ylabel("psi_desierd")
ax1.grid(True)

ax2.plot(data["time"],data["psi"],marker='o',linestyle='-',color='b')
ax2.set_title("psi")
ax2.set_xlabel("time")
ax2.set_ylabel("psi")
ax2.grid(True)

fig5, (ax1,ax2) = plt.subplots(1,2,figsize = (12,10))
ax1.plot(data["time"],data["x"],marker='o',linestyle='-',color='b')
ax1.set_title("x")
ax1.set_xlabel("time")
ax1.set_ylabel("x")
ax1.grid(True)

ax2.plot(data["time"],data["z"],marker='o',linestyle='-',color='b')
ax2.set_title("z")
ax2.set_xlabel("time")
ax2.set_ylabel("z")
ax2.grid(True)

fig6, (ax1,ax2) = plt.subplots(1,2,figsize = (12,10))
ax1.plot(data["time"],data["volt_desierd"],marker='o',linestyle='-',color='b')
ax1.set_title("volt_desierd")
ax1.set_xlabel("time")
ax1.set_ylabel("volt_desierd")
ax1.grid(True)

ax2.plot(data["time"],data["omega"],marker='o',linestyle='-',color='b')
ax2.set_title("omega")
ax2.set_xlabel("time")
ax2.set_ylabel("omega")
ax2.grid(True)


plt.show()
