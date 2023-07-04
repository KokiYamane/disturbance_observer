import pandas as pd
import matplotlib.pyplot as plt


def main():
    df = pd.read_csv('output.csv')
    print(df)

    fig, ax = plt.subplots(4, 1, figsize=(10, 10))

    ax[0].plot(df['time'], df['pos_cmd'],
               label=r'$q_{cmd}$', ls='-.', color='gray')
    ax[0].plot(df['time'], df['pos_res'],
               label=r'$q_{res}$', ls='-', color='black')
    # ax[0].set_xlabel('t [s]')
    ax[0].set_ylabel(r'$q_{res}$ [m]')
    ax[0].legend()
    ax[0].grid()

    ax[1].plot(df['time'], df['vel_res'],
               label=r'$\dot{q}_{res}$', ls='-', color='black')
    # ax[1].set_xlabel('t [s]')
    ax[1].set_ylabel(r'$\dot{q}_{res}$ [m/s]')
    ax[1].legend()
    ax[1].grid()

    ax[2].plot(df['time'], df['tau_ref'],
               label=r'$\tau_{ref}$', ls='-', color='black')
    # ax[1].set_xlabel('t [s]')
    ax[2].set_ylabel(r'$\tau_{ref}$ [N]')
    ax[2].legend()
    ax[2].grid()

    ax[3].plot(df['time'], df['tau_dis'],
               label=r'$\tau_{dis}$', ls='-.', color='gray')
    ax[3].plot(df['time'], df['tau_dis_hat'],
               label=r'$\hat{\tau}_{dis}$', ls='-', color='black')
    ax[3].set_xlabel('t [s]')
    ax[3].set_ylabel(r'$\tau_{dis}$ [N]')
    ax[3].legend()
    ax[3].grid()

    fig.align_ylabels()
    fig.savefig('result.png')


if __name__ == '__main__':
    main()
