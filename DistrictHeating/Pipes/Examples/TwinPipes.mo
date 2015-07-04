within DistrictHeating.Pipes.Examples;
model TwinPipes

  extends Modelica.Icons.Example;
  DoublePlugPipes.TwinPipeGround plug(
    redeclare package Medium = Annex60.Media.Water,
    L=100,
    Do=plug.Di,
    Di=0.02) annotation (Placement(transformation(extent={{-10,38},{10,66}})));
  IDEAS.Fluid.Sources.FixedBoundary bou2(
    T=273.15 + 70,
    redeclare package Medium = IDEAS.Media.Water.Simple,
    use_T=false,
    nPorts=1) annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-80,48})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort T1PlugIn(m_flow_nominal=0.1,
      redeclare package Medium = IDEAS.Media.Water.Simple) annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-28,70})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort T2PlugIn(m_flow_nominal=0.1,
      redeclare package Medium = IDEAS.Media.Water.Simple) annotation (
      Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={42,38})));
  Annex60.Fluid.Movers.FlowControlled_m_flow fan(redeclare package Medium =
        Annex60.Media.Water, m_flow_nominal=0.5)
    annotation (Placement(transformation(extent={{-62,60},{-42,80}})));
  Modelica.Blocks.Sources.Pulse pulse(
    width=5,
    period=86400,
    offset=273.15 + 50,
    amplitude=20)
    annotation (Placement(transformation(extent={{-82,86},{-90,94}})));
  Annex60.Fluid.HeatExchangers.HeaterCooler_T    idealHeater(
    dp_nominal=0,
    redeclare package Medium = Annex60.Media.Water,
    m_flow_nominal=0.1)
    annotation (Placement(transformation(extent={{-88,60},{-68,80}})));
  Annex60.Fluid.HeatExchangers.HeaterCooler_T    idealHeater1(
    dp_nominal=0,
    redeclare package Medium = Annex60.Media.Water,
    m_flow_nominal=0.1)
    annotation (Placement(transformation(extent={{84,28},{64,48}})));
  Modelica.Blocks.Sources.Pulse pulse1(
    period=86400,
    startTime=7200,
    width=60,
    amplitude=-0.099,
    offset=0.1)
    annotation (Placement(transformation(extent={{-74,86},{-66,94}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort T1PlugOut(
    m_flow_nominal=0.1,
    redeclare package Medium = IDEAS.Media.Water.Simple,
    tau=0) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={26,70})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort T2PlugOut(m_flow_nominal=0.1,
      redeclare package Medium = IDEAS.Media.Water.Simple) annotation (
      Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={-26,40})));
  Modelica.Blocks.Sources.Constant const(k=273.15 + 40)
    annotation (Placement(transformation(extent={{42,58},{50,66}})));
  Modelica.Blocks.Sources.Constant const1(k=273.15 + 30) annotation (
      Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=0,
        origin={-12,20})));
  DoublePipes.TwinPipeGround pipe(
    redeclare package Medium = Annex60.Media.Water,
    Pipe1(lambdaIns=0.026),
    Pipe2(lambdaIns=0.026),
    L=plug.L,
    lambdaI=plug.lambdaI,
    Di=plug.Di,
    Do=plug.Di,
    nSeg=50) annotation (Placement(transformation(extent={{-12,-24},{8,4}})));
  IDEAS.Fluid.Sources.FixedBoundary bou1(
    T=273.15 + 70,
    redeclare package Medium = IDEAS.Media.Water.Simple,
    use_T=false,
    nPorts=1) annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-80,-20})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort T1PipeIn(m_flow_nominal=0.1,
      redeclare package Medium = IDEAS.Media.Water.Simple) annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-28,6})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort T2PipeIn(m_flow_nominal=0.1,
      redeclare package Medium = IDEAS.Media.Water.Simple) annotation (
      Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={40,-24})));
  Annex60.Fluid.Movers.FlowControlled_m_flow fan1(
                                                 redeclare package Medium =
        Annex60.Media.Water, m_flow_nominal=0.5)
    annotation (Placement(transformation(extent={{-62,-4},{-42,16}})));
  Annex60.Fluid.HeatExchangers.HeaterCooler_T    idealHeater2(
    dp_nominal=0,
    redeclare package Medium = Annex60.Media.Water,
    m_flow_nominal=0.1)
    annotation (Placement(transformation(extent={{-88,-4},{-68,16}})));
  Annex60.Fluid.HeatExchangers.HeaterCooler_T    idealHeater3(
    dp_nominal=0,
    redeclare package Medium = Annex60.Media.Water,
    m_flow_nominal=0.1)
    annotation (Placement(transformation(extent={{64,-4},{84,16}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort T1PipeOut(
    m_flow_nominal=0.1,
    redeclare package Medium = IDEAS.Media.Water.Simple,
    tau=0) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={34,6})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort T2PipeOut(m_flow_nominal=0.1,
      redeclare package Medium = IDEAS.Media.Water.Simple) annotation (
      Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={-26,-28})));
  DoublePipes.TwinPipeGroundTR pipeTR(
    redeclare package Medium = Annex60.Media.Water,
    Pipe1(lambdaIns=0.026),
    Pipe2(lambdaIns=0.026),
    L=plug.L,
    lambdaI=plug.lambdaI,
    Di=plug.Di,
    Do=plug.Di,
    nSeg=pipe.nSeg)
    annotation (Placement(transformation(extent={{-12,-86},{8,-58}})));
  IDEAS.Fluid.Sources.FixedBoundary bou3(
    T=273.15 + 70,
    redeclare package Medium = IDEAS.Media.Water.Simple,
    use_T=false,
    nPorts=1) annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-82,-82})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort T1TRIn(m_flow_nominal=0.1, redeclare
      package Medium = IDEAS.Media.Water.Simple) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-30,-56})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort T2TRIn(m_flow_nominal=0.1, redeclare
      package Medium = IDEAS.Media.Water.Simple) annotation (Placement(
        transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={38,-86})));
  Annex60.Fluid.Movers.FlowControlled_m_flow fan2(
                                                 redeclare package Medium =
        Annex60.Media.Water, m_flow_nominal=0.5)
    annotation (Placement(transformation(extent={{-64,-66},{-44,-46}})));
  Annex60.Fluid.HeatExchangers.HeaterCooler_T    idealHeater4(
    dp_nominal=0,
    redeclare package Medium = Annex60.Media.Water,
    m_flow_nominal=0.1)
    annotation (Placement(transformation(extent={{-90,-66},{-70,-46}})));
  Annex60.Fluid.HeatExchangers.HeaterCooler_T    idealHeater5(
    dp_nominal=0,
    redeclare package Medium = Annex60.Media.Water,
    m_flow_nominal=0.1)
    annotation (Placement(transformation(extent={{62,-66},{82,-46}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort T1TROut(
    m_flow_nominal=0.1,
    redeclare package Medium = IDEAS.Media.Water.Simple,
    tau=0) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={32,-56})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort T2TROut(m_flow_nominal=0.1, redeclare
      package Medium = IDEAS.Media.Water.Simple) annotation (Placement(
        transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={-28,-90})));
  Modelica.Blocks.Sources.Pulse pulse2(
    period=86400,
    startTime=7200,
    width=60,
    offset=0.1,
    amplitude=-0.09)
    annotation (Placement(transformation(extent={{-74,22},{-66,30}})));
equation

  connect(fan.port_b, T1PlugIn.port_a) annotation (Line(
      points={{-42,70},{-38,70}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(idealHeater.port_b,fan. port_a) annotation (Line(
      points={{-68,70},{-62,70}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(T1PlugIn.port_b, plug.port_a1) annotation (Line(
      points={{-18,70},{-14,70},{-14,58},{-10,58}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(pulse.y,idealHeater. TSet) annotation (Line(
      points={{-90.4,90},{-96,90},{-96,76},{-90,76}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(T2PlugIn.port_b, plug.port_a2) annotation (Line(
      points={{32,38},{14,38},{14,46},{10,46}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(plug.port_b1, T1PlugOut.port_a) annotation (Line(
      points={{10,58},{14,58},{14,70},{16,70}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(plug.port_b2, T2PlugOut.port_a) annotation (Line(
      points={{-10,46},{-14,46},{-14,40},{-16,40}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(const.y,idealHeater1. TSet) annotation (Line(
      points={{50.4,62},{94,62},{94,44},{86,44}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(T1PlugOut.port_b, idealHeater1.port_a) annotation (Line(
      points={{36,70},{96,70},{96,38},{84,38}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(idealHeater1.port_b, T2PlugIn.port_a) annotation (Line(
      points={{64,38},{52,38}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(T2PlugOut.port_b, idealHeater.port_a) annotation (Line(
      points={{-36,40},{-90,40},{-90,70},{-88,70}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(pulse1.y,fan. m_flow_in) annotation (Line(
      points={{-65.6,90},{-52.2,90},{-52.2,82}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(fan1.port_b, T1PipeIn.port_a) annotation (Line(
      points={{-42,6},{-38,6}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(idealHeater2.port_b, fan1.port_a) annotation (Line(
      points={{-68,6},{-62,6}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(T1PipeIn.port_b, pipe.port_a1) annotation (Line(
      points={{-18,6},{-14,6},{-14,-4},{-12,-4}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(pipe.port_b2, T2PipeOut.port_a) annotation (Line(
      points={{-12,-16},{-14,-16},{-14,-28},{-16,-28}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(T1PipeOut.port_b, idealHeater3.port_a) annotation (Line(
      points={{44,6},{64,6}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(idealHeater3.port_b, T2PipeIn.port_a) annotation (Line(
      points={{84,6},{90,6},{90,-24},{50,-24}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(T2PipeOut.port_b, idealHeater2.port_a) annotation (Line(
      points={{-36,-28},{-90,-28},{-90,6},{-88,6}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(pipe.port_b1, T1PipeOut.port_a) annotation (Line(
      points={{8,-4},{16,-4},{16,6},{24,6}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(pipe.port_a2, T2PipeIn.port_b) annotation (Line(
      points={{8,-16},{20,-16},{20,-24},{30,-24}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(pipe.Tg, plug.Tg) annotation (Line(
      points={{-2,-24.2},{-2,-30},{12,-30},{12,20},{0,20},{0,37.8}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(const1.y, plug.Tg) annotation (Line(
      points={{-7.6,20},{0,20},{0,37.8}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(idealHeater2.TSet, idealHeater.TSet) annotation (Line(
      points={{-90,12},{-96,12},{-96,76},{-90,76}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(bou2.ports[1], idealHeater.port_a) annotation (Line(
      points={{-80,44},{-80,40},{-90,40},{-90,70},{-88,70}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(bou1.ports[1], idealHeater2.port_a) annotation (Line(
      points={{-80,-24},{-80,-28},{-90,-28},{-90,6},{-88,6}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(fan2.port_b, T1TRIn.port_a) annotation (Line(
      points={{-44,-56},{-40,-56}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(idealHeater4.port_b, fan2.port_a) annotation (Line(
      points={{-70,-56},{-64,-56}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(T1TRIn.port_b, pipeTR.port_a1) annotation (Line(
      points={{-20,-56},{-16,-56},{-16,-66},{-12,-66}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(pipeTR.port_b2, T2TROut.port_a) annotation (Line(
      points={{-12,-78},{-16,-78},{-16,-90},{-18,-90}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(T1TROut.port_b, idealHeater5.port_a) annotation (Line(
      points={{42,-56},{62,-56}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(idealHeater5.port_b, T2TRIn.port_a) annotation (Line(
      points={{82,-56},{88,-56},{88,-86},{48,-86}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(T2TROut.port_b, idealHeater4.port_a) annotation (Line(
      points={{-38,-90},{-92,-90},{-92,-56},{-90,-56}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(pipeTR.port_b1, T1TROut.port_a) annotation (Line(
      points={{8,-66},{14,-66},{14,-56},{22,-56}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(pipeTR.port_a2, T2TRIn.port_b) annotation (Line(
      points={{8,-78},{18,-78},{18,-86},{28,-86}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(pipeTR.Tg, plug.Tg) annotation (Line(
      points={{-2,-86.2},{-2,-94},{12,-94},{12,20},{0,20},{0,37.8}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(const.y, idealHeater3.TSet) annotation (Line(
      points={{50.4,62},{56,62},{56,12},{62,12}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(idealHeater5.TSet, idealHeater3.TSet) annotation (Line(
      points={{60,-50},{56,-50},{56,12},{62,12}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(idealHeater4.TSet, idealHeater.TSet) annotation (Line(
      points={{-92,-50},{-96,-50},{-96,76},{-90,76}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(fan2.m_flow_in, fan.m_flow_in) annotation (Line(
      points={{-54.2,-44},{-54.2,-42},{-54,-42},{-54,-38},{-60,-38},{-60,90},{
          -52.2,90},{-52.2,82}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(bou3.ports[1], idealHeater4.port_a) annotation (Line(
      points={{-82,-86},{-82,-90},{-92,-90},{-92,-56},{-90,-56}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(pulse2.y, fan1.m_flow_in) annotation (Line(
      points={{-65.6,26},{-52.2,26},{-52.2,18}},
      color={0,0,127},
      smooth=Smooth.None));
  annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{100,100}}), graphics),
    experiment(StopTime=400000, Interval=10),
    __Dymola_experimentSetupOutput);
end TwinPipes;
