within DistrictHeating.Pipes.Examples;
model MassflowPulse
  import DistrictHeating;
  extends Modelica.Icons.Example;
  import IDEAS;

  IDEAS.Fluid.Sources.FixedBoundary bou2(
    p=100000,
    T=273.15 + 70,
    redeclare package Medium = IDEAS.Media.Water.Simple,
    nPorts=1) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-88,0})));

  IDEAS.Fluid.Sources.FixedBoundary bou3(
    use_T=false,
    use_p=true,
    p=100000,
    nPorts=1,
    redeclare package Medium = IDEAS.Media.Water.Simple)
              annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={88,0})));

  IDEAS.Fluid.Sensors.TemperatureTwoPort TPlugIn(m_flow_nominal=0.1, redeclare
      package Medium = IDEAS.Media.Water.Simple) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-36,0})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TPlugOut(m_flow_nominal=0.1,
      redeclare package Medium = IDEAS.Media.Water.Simple) annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={58,0})));

  DistrictHeating.Pipes.PlugFlowHeatLosses plug(
    D=0.05,
    redeclare package Medium = Annex60.Media.Water,
    m_flow_nominal=1,
    dp_nominal=0,
    k=0.026,
    L=50)
    annotation (Placement(transformation(extent={{-22,-10},{-2,10}})));
  Buildings.HeatTransfer.Sources.FixedTemperature fixedTemperature(T=273.15 + 20)
    annotation (Placement(transformation(extent={{-96,60},{-76,80}})));
  Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor temperatureSensor
    annotation (Placement(transformation(extent={{-36,34},{-28,42}})));
  IDEAS.Fluid.Sources.FixedBoundary bou4(
    p=100000,
    T=273.15 + 70,
    redeclare package Medium = IDEAS.Media.Water.Simple,
    nPorts=1) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-88,-40})));
  IDEAS.Fluid.Sources.FixedBoundary bou5(
    use_T=false,
    use_p=true,
    p=100000,
    nPorts=1,
    redeclare package Medium = IDEAS.Media.Water.Simple)
              annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={88,-40})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TMocIn(m_flow_nominal=0.1, redeclare
      package Medium = IDEAS.Media.Water.Simple) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-34,-40})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TMocOut(m_flow_nominal=0.1, redeclare
      package Medium = IDEAS.Media.Water.Simple) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={58,-40})));
  Modelica.Fluid.Pipes.DynamicPipe MSL(
    length=plug.L,
    diameter=plug.D,
    use_HeatTransfer=true,
    redeclare package Medium = Annex60.Media.Water,
    T_start=273.15 + 20,
    redeclare model HeatTransfer =
        Modelica.Fluid.Pipes.BaseClasses.HeatTransfer.ConstantFlowHeatTransfer
        (T_ambient=273.15 + 20, alpha0=plug.k*plug.S/(plug.pi*plug.D)),
    nNodes=100)
    annotation (Placement(transformation(extent={{-6,-50},{14,-30}})));
  Modelica.Thermal.HeatTransfer.Components.ThermalCollector colAllToOne(m=MSL.nNodes)
    "Connector to assign multiple heat ports to one heat port" annotation (
      Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=180,
        origin={4,-26})));
  IDEAS.Fluid.Sources.FixedBoundary bou6(
    p=100000,
    T=273.15 + 70,
    redeclare package Medium = IDEAS.Media.Water.Simple,
    nPorts=1) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-88,-80})));
  IDEAS.Fluid.Sources.FixedBoundary bou7(
    use_T=false,
    use_p=true,
    p=100000,
    nPorts=1,
    redeclare package Medium = IDEAS.Media.Water.Simple)
              annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={88,-80})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TVolIn(m_flow_nominal=0.1, redeclare
      package Medium = IDEAS.Media.Water.Simple) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-34,-80})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TVolOut(m_flow_nominal=0.1, redeclare
      package Medium = IDEAS.Media.Water.Simple) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={58,-80})));
  Annex60.Fluid.MixingVolumes.MixingVolume vol(
    redeclare package Medium = Annex60.Media.Water,
    m_flow_nominal=1,
    nPorts=2,
    V=plug.V)
    annotation (Placement(transformation(extent={{34,-80},{14,-60}})));
  Modelica.Thermal.HeatTransfer.Components.ThermalResistor thermalResistor1(R=
        plug.r/plug.L) annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={40,50})));
  Annex60.Fluid.Movers.FlowControlled_m_flow fan(redeclare package Medium =
        Annex60.Media.Water, m_flow_nominal=0.5)
    annotation (Placement(transformation(extent={{-70,-10},{-50,10}})));
  Annex60.Fluid.Movers.FlowControlled_m_flow fan1(redeclare package Medium =
        Annex60.Media.Water, m_flow_nominal=0.5)
    annotation (Placement(transformation(extent={{-70,-50},{-50,-30}})));
  Annex60.Fluid.Movers.FlowControlled_m_flow fan2(redeclare package Medium =
        Annex60.Media.Water, m_flow_nominal=0.5)
    annotation (Placement(transformation(extent={{-70,-90},{-50,-70}})));
  Modelica.Blocks.Sources.Pulse pulse(
    width=10,
    period=1000,
    amplitude=0.9,
    offset=0.1)
    annotation (Placement(transformation(extent={{-100,20},{-80,40}})));
equation
  connect(TPlugOut.port_b, bou3.ports[1]) annotation (Line(
      points={{68,0},{78,0}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(TPlugIn.port_b, plug.port_a) annotation (Line(
      points={{-26,0},{-22,0}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(plug.port_b, TPlugOut.port_a) annotation (Line(
      points={{-2,0},{48,0}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(fixedTemperature.port, temperatureSensor.port) annotation (Line(
      points={{-76,70},{-62,70},{-62,38},{-36,38}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(temperatureSensor.T, plug.TBoundary) annotation (Line(
      points={{-28,38},{-11.8,38},{-11.8,5}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(TMocOut.port_b, bou5.ports[1]) annotation (Line(
      points={{68,-40},{78,-40}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(TMocIn.port_b, MSL.port_a) annotation (Line(
      points={{-24,-40},{-6,-40}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(MSL.port_b, TMocOut.port_a) annotation (Line(
      points={{14,-40},{48,-40}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(colAllToOne.port_a, MSL.heatPorts) annotation (Line(
      points={{4,-32},{4,-35.6},{4.1,-35.6}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(colAllToOne.port_b, temperatureSensor.port) annotation (Line(
      points={{4,-20},{4,70},{-62,70},{-62,38},{-36,38}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(TVolOut.port_b, bou7.ports[1]) annotation (Line(
      points={{68,-80},{78,-80}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(TVolIn.port_b, vol.ports[1]) annotation (Line(
      points={{-24,-80},{26,-80}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(vol.ports[2], TVolOut.port_a) annotation (Line(
      points={{22,-80},{48,-80}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(vol.heatPort, thermalResistor1.port_b) annotation (Line(
      points={{34,-70},{40,-70},{40,40}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(thermalResistor1.port_a, temperatureSensor.port) annotation (Line(
      points={{40,60},{40,70},{-62,70},{-62,38},{-36,38}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(bou2.ports[1], fan.port_a) annotation (Line(
      points={{-78,0},{-70,0}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(fan.port_b, TPlugIn.port_a) annotation (Line(
      points={{-50,0},{-46,0}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(bou4.ports[1], fan1.port_a) annotation (Line(
      points={{-78,-40},{-70,-40}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(TMocIn.port_a, fan1.port_b) annotation (Line(
      points={{-44,-40},{-50,-40}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(bou6.ports[1], fan2.port_a) annotation (Line(
      points={{-78,-80},{-70,-80}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(TVolIn.port_a, fan2.port_b) annotation (Line(
      points={{-44,-80},{-50,-80}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(fan1.m_flow_in, fan2.m_flow_in) annotation (Line(
      points={{-60.2,-28},{-60.2,-20},{-68,-20},{-68,-60},{-60.2,-60},{-60.2,
          -68}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(pulse.y, fan.m_flow_in) annotation (Line(
      points={{-79,30},{-60.2,30},{-60.2,12}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(pulse.y, fan2.m_flow_in) annotation (Line(
      points={{-79,30},{-68,30},{-68,-60},{-60.2,-60},{-60.2,-68}},
      color={0,0,127},
      smooth=Smooth.None));
  annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{100,100}}), graphics), Icon(coordinateSystem(extent={{-100,
            -100},{100,100}})),
    experiment(StopTime=25000),
    __Dymola_experimentSetupOutput,
    __Dymola_Commands(executeCall(ensureSimulated=true) = RunScript(
        "C:/Users/u0098668/Documents/Modelica/DistrictHeating/DistrictHeating/simulate and plot.mos")
        "Simulate and plot"));
end MassflowPulse;
