Imports System.IO
Imports VehicleSim

Module Steer_Control
  Sub Main(ByVal cmdArgs() As String)
    Dim n_import, n_export, status, ibarg, print_interval As Integer
    Dim t_start, t_stop, t_step, t_current, DR As Double
    Dim Lfwd, GainStr, LatTrack As Double
    Dim Xpreview, Ypreview, Xcg, Ycg, Yaw, road_l As Double

    DR = 180.0 / Math.PI ' scale factor: degrees per radian

    Console.WriteLine("VB Program with custom steer controller.")
    Console.WriteLine("")

    Dim pathToSimfile As String = Path.Combine(Directory.GetCurrentDirectory(), "simfile.sim")
    If cmdArgs.Length = 1 Then
      pathToSimfile = cmdArgs(0)
    End If

    Dim vs As Simulation = New Simulation()

    Dim pathToVsDll = vs.get_dll_path(pathToSimfile)

    If pathToVsDll IsNot String.Empty And File.Exists(pathToVsDll) Then
      Dim vsDll As IntPtr = vs.LoadLibrary(pathToVsDll)
      vs.get_api(vsDll)

      ' Call vs_read_configuration to read parsfile and initialize the VS solver.
      ' Get no. of import/export variables, start time, stop time and time step.
      vs.ReadConfiguration(pathToSimfile, n_import, n_export, t_start, t_stop, t_step)
      t_current = t_start

      ' Create import and export arrays based on sizes from VS solver
      Dim import(0 To n_import - 1) As Double
      Dim export(0 To n_export - 1) As Double

      vs.CopyExportVars(export(0)) ' get export variables from vs solver

      ' New parameters for use in the steer controller
      Lfwd = 20.0
      GainStr = 10 ' This has units deg/m
      LatTrack = -1.6

      ' constants to show progress bar
      print_interval = (t_stop - t_start) / t_step / 50
      ibarg = print_interval

      status = 0

      ' Run the integration loop
      Do While (status = 0)
        t_current = t_current + t_step ' increment the time

        ' Steering Controller variables, based on previous exports
        Xcg = export(0)
        Ycg = export(1)
        Yaw = export(2) / DR ' convert export deg to rad
        Xpreview = Xcg + Lfwd * Math.Cos(Yaw)
        Ypreview = Ycg + Lfwd * Math.Sin(Yaw)
        road_l = vs.GetRoadL(Xpreview, Ypreview)

        ' copy values for 3 variables that the VS solver will import
        import(0) = GainStr * (LatTrack - road_l) ' This has units of deg
        import(1) = Xpreview
        import(2) = Ypreview

        ' Call the VS API integration function
        status = vs.IntegrateIO(t_current, import(0), export(0))

        ' Update bar graph to show progress
        If ibarg >= print_interval Then
          Console.Write("=")
          ibarg = 0
        End If
        ibarg = ibarg + 1
      Loop

      ' Terminate solver
      vs.TerminateRun(t_current)
      End If
  End Sub
End Module
