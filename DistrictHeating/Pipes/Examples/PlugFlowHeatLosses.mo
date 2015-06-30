within DistrictHeating.Pipes.Examples;
model PlugFlowHeatLosses
  import DistrictHeating;
  extends Modelica.Icons.Example;
  import IDEAS;

  IDEAS.Fluid.Sources.FixedBoundary bou2(
    nPorts=1,
    p=100000,
    T=273.15 + 70,
    redeclare package Medium = IDEAS.Media.Water.Simple)
              annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-86,-20})));

  IDEAS.Fluid.Sources.FixedBoundary bou3(
    use_T=false,
    use_p=true,
    p=100000,
    nPorts=1,
    redeclare package Medium = IDEAS.Media.Water.Simple)
              annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={90,-20})));

  IDEAS.Fluid.Movers.Pump pump3(redeclare package Medium =
        IDEAS.Media.Water.Simple, m_flow_nominal=0.1)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-60,-20})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TPlugIn(m_flow_nominal=0.1, redeclare
      package Medium = IDEAS.Media.Water.Simple) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-34,-20})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TPlugOut(m_flow_nominal=0.1,
      redeclare package Medium = IDEAS.Media.Water.Simple) annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={60,-20})));

  DistrictHeating.Pipes.PlugFlowHeatLosses plug(
    D=0.05,
    redeclare package Medium = Annex60.Media.Water,
    m_flow_nominal=1,
    dp_nominal=0,
    L=100,
    k=0.026)
    annotation (Placement(transformation(extent={{-20,-30},{0,-10}})));
  Buildings.HeatTransfer.Sources.FixedTemperature fixedTemperature(T=273.15 + 20)
    annotation (Placement(transformation(extent={{-94,30},{-74,50}})));
  Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor temperatureSensor
    annotation (Placement(transformation(extent={{-34,4},{-26,12}})));
  IDEAS.Fluid.Sources.FixedBoundary bou1(
    p=100000,
    T=273.15 + 70,
    redeclare package Medium = IDEAS.Media.Water.Simple,
    use_T=false)
              annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={40,66})));
  IDEAS.Fluid.Sources.FixedBoundary bou4(
    nPorts=1,
    p=100000,
    T=273.15 + 70,
    redeclare package Medium = IDEAS.Media.Water.Simple)
              annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-86,-54})));
  IDEAS.Fluid.Sources.FixedBoundary bou5(
    use_T=false,
    use_p=true,
    p=100000,
    nPorts=1,
    redeclare package Medium = IDEAS.Media.Water.Simple)
              annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={90,-54})));
  IDEAS.Fluid.Movers.Pump pump1(redeclare package Medium =
        IDEAS.Media.Water.Simple, m_flow_nominal=0.1)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-60,-54})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TMocIn(m_flow_nominal=0.1, redeclare
      package Medium = IDEAS.Media.Water.Simple) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-32,-54})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TMocOut(m_flow_nominal=0.1, redeclare
      package Medium = IDEAS.Media.Water.Simple) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={60,-54})));
  Modelica.Fluid.Pipes.DynamicPipe MSL(
    length=plug.L,
    diameter=plug.D,
    use_HeatTransfer=true,
    nNodes=50,
    redeclare package Medium = Annex60.Media.Water,
    T_start=273.15 + 20,
    redeclare model HeatTransfer =
        Modelica.Fluid.Pipes.BaseClasses.HeatTransfer.ConstantFlowHeatTransfer
        (T_ambient=273.15 + 20, alpha0=plug.k*plug.S/(plug.pi*plug.D)))
    annotation (Placement(transformation(extent={{-4,-64},{16,-44}})));
  Modelica.Thermal.HeatTransfer.Components.ThermalCollector colAllToOne(m=50)
    "Connector to assign multiple heat ports to one heat port" annotation (
      Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=180,
        origin={6,-42})));
  IDEAS.Fluid.Sources.FixedBoundary bou6(
    nPorts=1,
    p=100000,
    T=273.15 + 70,
    redeclare package Medium = IDEAS.Media.Water.Simple)
              annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-86,-84})));
  IDEAS.Fluid.Sources.FixedBoundary bou7(
    use_T=false,
    use_p=true,
    p=100000,
    nPorts=1,
    redeclare package Medium = IDEAS.Media.Water.Simple)
              annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={90,-84})));
  IDEAS.Fluid.Movers.Pump pump2(redeclare package Medium =
        IDEAS.Media.Water.Simple, m_flow_nominal=0.1)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-60,-84})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TVolIn(m_flow_nominal=0.1, redeclare
      package Medium = IDEAS.Media.Water.Simple) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-32,-84})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TVolOut(m_flow_nominal=0.1, redeclare
      package Medium = IDEAS.Media.Water.Simple) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={60,-84})));
  Annex60.Fluid.MixingVolumes.MixingVolume vol(
    redeclare package Medium = Annex60.Media.Water,
    m_flow_nominal=1,
    nPorts=2,
    V=plug.V)
    annotation (Placement(transformation(extent={{36,-84},{16,-64}})));
  Modelica.Thermal.HeatTransfer.Components.ThermalResistor thermalResistor1(R=
        plug.r/plug.L) annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={40,20})));
equation
  connect(pump3.port_b, TPlugIn.port_a) annotation (Line(
      points={{-50,-20},{-44,-20}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(pump3.port_a, bou2.ports[1]) annotation (Line(
      points={{-70,-20},{-76,-20}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(TPlugOut.port_b, bou3.ports[1]) annotation (Line(
      points={{70,-20},{80,-20}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(TPlugIn.port_b, plug.port_a) annotation (Line(
      points={{-24,-20},{-20,-20}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(plug.port_b, TPlugOut.port_a) annotation (Line(
      points={{0,-20},{50,-20}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(fixedTemperature.port, temperatureSensor.port) annotation (Line(
      points={{-74,40},{-60,40},{-60,8},{-34,8}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(temperatureSensor.T, plug.TBoundary) annotation (Line(
      points={{-26,8},{-9.8,8},{-9.8,-15}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(pump1.port_b, TMocIn.port_a) annotation (Line(
      points={{-50,-54},{-42,-54}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(TMocOut.port_b, bou5.ports[1]) annotation (Line(
      points={{70,-54},{80,-54}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(bou4.ports[1], pump1.port_a) annotation (Line(
      points={{-76,-54},{-70,-54}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(TMocIn.port_b, MSL.port_a) annotation (Line(
      points={{-22,-54},{-4,-54}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(MSL.port_b, TMocOut.port_a) annotation (Line(
      points={{16,-54},{50,-54}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(colAllToOne.port_a, MSL.heatPorts) annotation (Line(
      points={{6,-48},{6,-49.6},{6.1,-49.6}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(colAllToOne.port_b, temperatureSensor.port) annotation (Line(
      points={{6,-36},{6,40},{-60,40},{-60,8},{-48,8},{-48,8},{-34,8}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(pump2.port_b, TVolIn.port_a) annotation (Line(
      points={{-50,-84},{-42,-84}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(TVolOut.port_b, bou7.ports[1]) annotation (Line(
      points={{70,-84},{80,-84}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(bou6.ports[1],pump2. port_a) annotation (Line(
      points={{-76,-84},{-70,-84}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(TVolIn.port_b, vol.ports[1]) annotation (Line(
      points={{-22,-84},{28,-84}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(vol.ports[2], TVolOut.port_a) annotation (Line(
      points={{24,-84},{50,-84}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(vol.heatPort, thermalResistor1.port_b) annotation (Line(
      points={{36,-74},{40,-74},{40,10}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(thermalResistor1.port_a, temperatureSensor.port) annotation (Line(
      points={{40,30},{40,40},{-60,40},{-60,8},{-34,8}},
      color={191,0,0},
      smooth=Smooth.None));
  annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{100,100}}), graphics), Icon(coordinateSystem(extent={{-100,
            -100},{100,100}})),
    experiment(StopTime=25000),
    __Dymola_experimentSetupOutput);
end PlugFlowHeatLosses;
