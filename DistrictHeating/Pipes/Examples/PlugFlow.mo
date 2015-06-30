within DistrictHeating.Pipes.Examples;
model PlugFlow
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
        origin={-88,10})));

  IDEAS.Fluid.Sources.FixedBoundary bou3(
    use_T=false,
    use_p=true,
    p=100000,
    nPorts=1,
    redeclare package Medium = IDEAS.Media.Water.Simple)
              annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={88,10})));

  IDEAS.Fluid.Movers.Pump pump3(redeclare package Medium =
        IDEAS.Media.Water.Simple, m_flow_nominal=0.1,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    energyDynamics=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-62,10})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TPlugIn(m_flow_nominal=0.1, redeclare
      package Medium = IDEAS.Media.Water.Simple) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-36,10})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TPlugOut(m_flow_nominal=0.1,
      redeclare package Medium = IDEAS.Media.Water.Simple) annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={58,10})));

  DistrictHeating.Pipes.PlugFlowHeatLosses plug(
    D=0.05,
    redeclare package Medium = Annex60.Media.Water,
    m_flow_nominal=1,
    dp_nominal=0,
    L=100,
    k=0.026)
    annotation (Placement(transformation(extent={{-10,0},{10,20}})));
  Buildings.HeatTransfer.Sources.FixedTemperature fixedTemperature(T=273.15 + 20)
    annotation (Placement(transformation(extent={{-80,60},{-60,80}})));
  Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor temperatureSensor
    annotation (Placement(transformation(extent={{-40,60},{-20,80}})));
equation
  connect(pump3.port_b, TPlugIn.port_a) annotation (Line(
      points={{-52,10},{-46,10}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(pump3.port_a, bou2.ports[1]) annotation (Line(
      points={{-72,10},{-78,10}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(TPlugOut.port_b, bou3.ports[1]) annotation (Line(
      points={{68,10},{78,10}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(TPlugIn.port_b, plug.port_a) annotation (Line(
      points={{-26,10},{-10,10}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(plug.port_b, TPlugOut.port_a) annotation (Line(
      points={{10,10},{48,10}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(fixedTemperature.port, temperatureSensor.port) annotation (Line(
      points={{-60,70},{-40,70}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(temperatureSensor.T, plug.TBoundary) annotation (Line(
      points={{-20,70},{0.2,70},{0.2,15}},
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
end PlugFlow;
