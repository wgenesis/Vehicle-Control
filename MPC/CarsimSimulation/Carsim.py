import Simulation

from MPC import *

if __name__ == '__main__':
    print('START')

    platform = sys.platform;
    degrees_to_radians_scale_factor = 180.0 / math.pi
   
    parser = argparse.ArgumentParser(
        description='Python 3.5 script which runs the steering control example')

    parser.add_argument('--simfile', '-s', dest='path_to_sim_file',
                        default=os.path.join(os.getcwd(), "simfile.sim"),
                        help="Path to simfile.")
    args = parser.parse_args()
    sim_file_filename = args.path_to_sim_file
    vs = Simulation.VehicleSimulation()
    path_to_vs_dll = vs.get_dll_path(sim_file_filename)

    #画图各项参数
    v_list=[]
    theta_list=[]
    real_x=[]
    real_y=[]
    out_x=[]
    out_y=[]
    MPC = ModelPredictiveControl()
    MPC.loadTraj('./Extensions/Custom_Py/UTM.csv')

    if path_to_vs_dll is not None and os.path.exists(path_to_vs_dll):
        vs_dll = ctypes.cdll.LoadLibrary(path_to_vs_dll)
        if vs_dll is not None:
            if vs.get_api(vs_dll):
                # Call vs_read_configuration to read parsfile and initialize the VS solver.
                # Get no. of import/export variables, start time, stop time and time step.
                configuration = vs.ReadConfiguration(sim_file_filename)
                t_current = configuration.get('t_start')

                # Create import and export arrays based on sizes from VS solver
                import_array = [0.0, 0.0, 0.0]
                export_array = []
                export_array = vs.CopyExportVars(configuration.get('n_export')) # get export variables from vs solver

                # New parameters for use in the steer controller
                Lfwd = 20.0
                GainStr = 10 # This has units deg/m
                LatTrack = -2.0

                # constants to show progress bar
                print_interval = (configuration.get('t_stop') - configuration.get('t_start')) / configuration.get('t_step') / 50
                ibarg = print_interval
                status = 0

                # Check that we have enough export variables
                last_t=0

                n = 0

                while status is 0:
                    t_current = t_current + configuration.get('t_step') # increment the time

                    if t_current-last_t>MPC.T:
                        out_x.append(MPC.traj[n]['x'])
                        out_y.append(MPC.traj[n]['y'])
                        MPC.carsimExport=export_array
                        MPC.predict(n)
                        n += 10
                        last_t=t_current
                        real_x.append(export_array[0])
                        real_y.append(export_array[1])
                    import_array =MPC.carsimImport
                    # Call the VS API integration function
                    status, export_array = vs.IntegrateIO(t_current, import_array, export_array)

                    ibarg = ibarg + 1

            # Terminate solver
            vs.TerminateRun(t_current)
    plt.figure(1)
    for p1_x,p1_y,p2_x,p2_y in zip(real_x,real_y,out_x, out_y):
        plt.plot([p1_x, p2_x], [p1_y, p2_y], 'r:')
    plt.plot(real_x,real_y, "*")
    plt.plot(out_x,out_y,'g')
    plt.figure(2)
    plt.plot(range(len(MPC.v_list)),MPC.v_list)
    plt.figure(3)
    plt.plot(range(len(MPC.theta_list)),MPC.theta_list)
    plt.figure(4)
    plt.plot(range(len(MPC.r_yaw_list)),MPC.r_yaw_list,'*')
    plt.plot(range(len(MPC.yaw_list)),MPC.yaw_list,'-')
    plt.plot(range(len(MPC.error_yaw_list)),MPC.error_yaw_list)
    plt.figure(5)
    plt.plot(range(len(MPC.out_v)),MPC.out_v)
    plt.axhline(y=MPC.lb[0],color='r',linestyle='--')
    plt.axhline(y=MPC.ub[0],color='r',linestyle='--')
    plt.figure(6)
    plt.plot(range(len(MPC.out_theta)), MPC.out_theta)
    plt.axhline(y=MPC.lb[1], color='r', linestyle='--')
    plt.axhline(y=MPC.ub[1], color='r', linestyle='--')
    plt.show()