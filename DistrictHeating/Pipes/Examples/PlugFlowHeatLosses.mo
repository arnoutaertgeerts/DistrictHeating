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
        origin={58,-20})));

  DistrictHeating.Pipes.PlugFlowHeatLosses plug(
    D=0.05,
    redeclare package Medium = Annex60.Media.Water,
    m_flow_nominal=1,
    dp_nominal=0,
    L=100,
    k=0.026)
    annotation (Placement(transformation(extent={{-10,-30},{10,-10}})));
  Buildings.HeatTransfer.Sources.FixedTemperature fixedTemperature(T=273.15 + 20)
    annotation (Placement(transformation(extent={{-94,30},{-74,50}})));
  Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor temperatureSensor
    annotation (Placement(transformation(extent={{-40,10},{-20,30}})));
  Annex60.Fluid.MixingVolumes.MixingVolume vol(
    redeclare package Medium = Annex60.Media.Water,
    m_flow_nominal=1,
    nPorts=1,
    T_start=273.15 + 70,
    V=plug.V)
    annotation (Placement(transformation(extent={{-10,70},{10,90}})));
  IDEAS.Fluid.Sources.FixedBoundary bou1(
    nPorts=1,
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
        origin={58,-54})));
  Modelica.Fluid.Pipes.DynamicPipe MSL(
    length=plug.L,
    diameter=plug.D,
    use_HeatTransfer=true,
    nNodes=50,
    redeclare package Medium = Annex60.Media.Water,
    T_start=273.15 + 20,
    redeclare model HeatTransfer =
        Modelica.Fluid.Pipes.BaseClasses.HeatTransfer.ConstantFlowHeatTransfer
        (
        k=plug.k*plug.S/(2*Modelica.Constants.pi*(plug.D + 2*plug.h)),
        T_ambient=273.15 + 20,
        alpha0=0))
    annotation (Placement(transformation(extent={{10,-64},{30,-44}})));
  Modelica.Thermal.HeatTransfer.Components.ThermalCollector colAllToOne(m=50)
    "Connector to assign multiple heat ports to one heat port" annotation (
      Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=180,
        origin={20,-42})));
  Modelica.Thermal.HeatTransfer.Components.ThermalResistor thermalResistor1
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-60,64})));
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
      points={{68,-20},{80,-20}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(TPlugIn.port_b, plug.port_a) annotation (Line(
      points={{-24,-20},{-10,-20}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(plug.port_b, TPlugOut.port_a) annotation (Line(
      points={{10,-20},{48,-20}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(fixedTemperature.port, temperatureSensor.port) annotation (Line(
      points={{-74,40},{-60,40},{-60,20},{-40,20}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(temperatureSensor.T, plug.TBoundary) annotation (Line(
      points={{-20,20},{0.2,20},{0.2,-15}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(vol.ports[1], bou1.ports[1]) annotation (Line(
      points={{0,70},{0,66},{30,66}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(pump1.port_b, TMocIn.port_a) annotation (Line(
      points={{-50,-54},{-42,-54}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(TMocOut.port_b, bou5.ports[1]) annotation (Line(
      points={{68,-54},{80,-54}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(bou4.ports[1], pump1.port_a) annotation (Line(
      points={{-76,-54},{-70,-54}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(TMocIn.port_b, MSL.port_a) annotation (Line(
      points={{-22,-54},{10,-54}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(MSL.port_b, TMocOut.port_a) annotation (Line(
      points={{30,-54},{48,-54}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(colAllToOne.port_a, MSL.heatPorts) annotation (Line(
      points={{20,-48},{20,-49.6},{20.1,-49.6}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(colAllToOne.port_b, temperatureSensor.port) annotation (Line(
      points={{20,-36},{20,40},{-60,40},{-60,20},{-40,20}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(thermalResistor1.port_b, vol.heatPort) annotation (Line(
      points={{-60,74},{-60,80},{-10,80}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(thermalResistor1.port_a, temperatureSensor.port) annotation (Line(
      points={{-60,54},{-60,20},{-40,20}},
      color={191,0,0},
      smooth=Smooth.None));
  annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{100,100}}), graphics), Icon(coordinateSystem(extent={{-100,
            -100},{100,100}})),
    experiment(StopTime=25000),
    __Dymola_experimentSetupOutput);
end PlugFlowHeatLosses;
