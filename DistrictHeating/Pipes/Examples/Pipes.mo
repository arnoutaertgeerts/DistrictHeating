within DistrictHeating.Pipes.Examples;
model Pipes
  import DistrictHeating;
  extends Modelica.Icons.Example;
  import IDEAS;

  Buildings.HeatTransfer.Sources.FixedTemperature fixedTemperature(T=273.15 + 20)
    annotation (Placement(transformation(extent={{-20,0},{0,20}})));
  IDEAS.Fluid.Sources.FixedBoundary bou4(
    p=100000,
    T=273.15 + 70,
    redeclare package Medium = IDEAS.Media.Water.Simple,
    nPorts=1) annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=0,
        origin={-94,-30})));
  IDEAS.Fluid.Sources.FixedBoundary bou5(
    use_T=false,
    use_p=true,
    p=100000,
    nPorts=1,
    redeclare package Medium = IDEAS.Media.Water.Simple)
              annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={90,-30})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TMocIn(m_flow_nominal=0.1, redeclare
      package Medium = IDEAS.Media.Water.Simple) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-20,-30})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TMocOut(m_flow_nominal=0.1, redeclare
      package Medium = IDEAS.Media.Water.Simple) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={60,-30})));
  Annex60.Fluid.Movers.FlowControlled_m_flow fan1(redeclare package Medium =
        Annex60.Media.Water, m_flow_nominal=0.5)
    annotation (Placement(transformation(extent={{-56,-40},{-36,-20}})));
  Modelica.Blocks.Sources.Pulse pulse(
    width=5,
    period=86400,
    offset=273.15 + 50,
    amplitude=20)
    annotation (Placement(transformation(extent={{-68,-16},{-76,-8}})));
  Annex60.Fluid.HeatExchangers.HeaterCooler_T    idealHeater1(
    dp_nominal=0,
    redeclare package Medium = Annex60.Media.Water,
    m_flow_nominal=0.1)
    annotation (Placement(transformation(extent={{-82,-40},{-62,-20}})));
  Modelica.Blocks.Sources.Pulse pulse1(
    period=86400,
    amplitude=0,
    width=0,
    offset=0.1)
    annotation (Placement(transformation(extent={{-64,-16},{-56,-8}})));
  Buildings.Fluid.FixedResistances.Pipe pip(
    redeclare package Medium = Annex60.Media.Water,
    m_flow_nominal=0.1,
    dp_nominal=0,
    thicknessIns=0.02,
    lambdaIns=0.026,
    diameter=0.05,
    length=100,
    nSeg=2) annotation (Placement(transformation(extent={{0,-40},{20,-20}})));
  Buildings.Fluid.FixedResistances.Pipe pip1(
    redeclare package Medium = Annex60.Media.Water,
    m_flow_nominal=0.1,
    dp_nominal=0,
    nSeg=pip.nSeg,
    thicknessIns=pip.thicknessIns,
    lambdaIns=pip.lambdaIns,
    diameter=pip.diameter,
    length=pip.length)
    annotation (Placement(transformation(extent={{26,-40},{46,-20}})));
equation
  connect(TMocOut.port_b,bou5. ports[1]) annotation (Line(
      points={{70,-30},{80,-30}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(TMocIn.port_a, fan1.port_b) annotation (Line(
      points={{-30,-30},{-36,-30}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(bou4.ports[1], idealHeater1.port_a) annotation (Line(
      points={{-90,-30},{-82,-30}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(idealHeater1.port_b, fan1.port_a) annotation (Line(
      points={{-62,-30},{-56,-30}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(pulse1.y, fan1.m_flow_in) annotation (Line(
      points={{-55.6,-12},{-46.2,-12},{-46.2,-18}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(pulse.y, idealHeater1.TSet) annotation (Line(
      points={{-76.4,-12},{-88,-12},{-88,-24},{-84,-24}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(TMocIn.port_b, pip.port_a) annotation (Line(
      points={{-10,-30},{0,-30}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(pip.port_b, pip1.port_a) annotation (Line(
      points={{20,-30},{26,-30}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(TMocOut.port_a, pip1.port_b) annotation (Line(
      points={{50,-30},{46,-30}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(fixedTemperature.port, pip.heatPort) annotation (Line(
      points={{0,10},{10,10},{10,-25}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(fixedTemperature.port, pip1.heatPort) annotation (Line(
      points={{0,10},{36,10},{36,-25}},
      color={191,0,0},
      smooth=Smooth.None));
  annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{100,100}}), graphics), Icon(coordinateSystem(extent={{-100,
            -100},{100,100}})),
    experiment(StopTime=604800),
    __Dymola_experimentSetupOutput);
end Pipes;
