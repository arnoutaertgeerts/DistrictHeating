within DistrictHeating.Pipes.Examples;
model PlugFlow
  import DistrictHeating;
  extends Modelica.Icons.Example;
  import IDEAS;

  IDEAS.Fluid.Sources.FixedBoundary bou2(
    p=100000,
    T=273.15 + 70,
    redeclare package Medium = IDEAS.Media.Water.Simple,
    nPorts=1) annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=0,
        origin={-94,10})));
  IDEAS.Fluid.Sources.FixedBoundary bou3(
    use_T=false,
    use_p=true,
    p=100000,
    nPorts=1,
    redeclare package Medium = IDEAS.Media.Water.Simple)
              annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={90,10})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TPlugIn(m_flow_nominal=0.1, redeclare
      package Medium = IDEAS.Media.Water.Simple,
    tau=1)                                       annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-22,10})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TPlugOut(m_flow_nominal=0.1,
      redeclare package Medium = IDEAS.Media.Water.Simple,
    tau=0)                                                 annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={60,10})));
  DistrictHeating.Pipes.PlugFlowHeatLosses plug(
    D=0.05,
    redeclare package Medium = Annex60.Media.Water,
    m_flow_nominal=1,
    dp_nominal=0,
    k=0.026,
    L=50)
    annotation (Placement(transformation(extent={{-6,0},{14,20}})));
  Buildings.HeatTransfer.Sources.FixedTemperature fixedTemperature(T=273.15 + 20)
    annotation (Placement(transformation(extent={{-94,70},{-74,90}})));
  Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor temperatureSensor
    annotation (Placement(transformation(extent={{-34,76},{-26,84}})));
  Annex60.Fluid.Movers.FlowControlled_m_flow fan(
    redeclare package Medium = Annex60.Media.Water,
    m_flow_nominal=0.5,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState)
    annotation (Placement(transformation(extent={{-56,0},{-36,20}})));
  Modelica.Blocks.Sources.Pulse pulse(
    width=5,
    period=86400,
    offset=273.15 + 50,
    amplitude=20)
    annotation (Placement(transformation(extent={{-68,30},{-76,38}})));
  Annex60.Fluid.HeatExchangers.HeaterCooler_T    idealHeater(
    dp_nominal=0,
    redeclare package Medium = Annex60.Media.Water,
    m_flow_nominal=0.1)
    annotation (Placement(transformation(extent={{-82,0},{-62,20}})));
  Modelica.Blocks.Sources.Pulse pulse1(
    period=86400,
    amplitude=0,
    width=0,
    offset=0.1)
    annotation (Placement(transformation(extent={{-60,30},{-52,38}})));
  DistrictHeating.Pipes.PlugFlowHeatLosses plug1(
    D=0.05,
    redeclare package Medium = Annex60.Media.Water,
    m_flow_nominal=1,
    dp_nominal=0,
    k=0.026,
    L=50)
    annotation (Placement(transformation(extent={{22,0},{42,20}})));
  Modelica.Blocks.Continuous.Filter filter(f_cut=1)
    annotation (Placement(transformation(extent={{-82,30},{-90,38}})));
equation
  connect(TPlugOut.port_b,bou3. ports[1]) annotation (Line(
      points={{70,10},{80,10}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(TPlugIn.port_b,plug. port_a) annotation (Line(
      points={{-12,10},{-6,10}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(fixedTemperature.port,temperatureSensor. port) annotation (Line(
      points={{-74,80},{-34,80}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(temperatureSensor.T,plug. TBoundary) annotation (Line(
      points={{-26,80},{4.2,80},{4.2,15}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(fan.port_b, TPlugIn.port_a) annotation (Line(
      points={{-36,10},{-32,10}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(bou2.ports[1], idealHeater.port_a) annotation (Line(
      points={{-90,10},{-82,10}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(idealHeater.port_b, fan.port_a) annotation (Line(
      points={{-62,10},{-56,10}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(plug.port_b, plug1.port_a) annotation (Line(
      points={{14,10},{22,10}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(plug1.port_b, TPlugOut.port_a) annotation (Line(
      points={{42,10},{50,10}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(plug1.TBoundary, plug.TBoundary) annotation (Line(
      points={{32.2,15},{32.2,80},{4.2,80},{4.2,15}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(pulse1.y, fan.m_flow_in) annotation (Line(
      points={{-51.6,34},{-46.2,34},{-46.2,22}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(pulse.y, filter.u) annotation (Line(
      points={{-76.4,34},{-81.2,34}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(filter.y, idealHeater.TSet) annotation (Line(
      points={{-90.4,34},{-94,34},{-94,16},{-84,16}},
      color={0,0,127},
      smooth=Smooth.None));
  annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{100,100}}), graphics), Icon(coordinateSystem(extent={{-100,
            -100},{100,100}})),
    experiment(StopTime=604800),
    __Dymola_experimentSetupOutput,
    __Dymola_Commands(executeCall(ensureSimulated=true) = RunScript(
        "C:/Users/u0098668/Documents/Modelica/DistrictHeating/DistrictHeating/simulate and plot.mos")
        "Simulate and plot"));
end PlugFlow;
