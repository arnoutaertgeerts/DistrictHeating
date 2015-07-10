within DistrictHeating.Pipes.Examples;
model BuildingTest
  import DistrictHeating;
  extends Modelica.Icons.Example;
  import IDEAS;

  Buildings.HeatTransfer.Sources.FixedTemperature fixedTemperature[2](T=273.15 + 20)
    annotation (Placement(transformation(extent={{-20,40},{0,60}})));
  IDEAS.Fluid.Sources.FixedBoundary bou4(
    p=100000,
    T=273.15 + 70,
    redeclare package Medium = IDEAS.Media.Water.Simple,
    nPorts=1) annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=0,
        origin={-94,16})));
  IDEAS.Fluid.Sources.FixedBoundary bou5(
    use_T=false,
    use_p=true,
    p=100000,
    nPorts=1,
    redeclare package Medium = IDEAS.Media.Water.Simple)
              annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={90,16})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TMocIn(m_flow_nominal=0.1, redeclare
      package Medium = IDEAS.Media.Water.Simple) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-20,16})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TMocOut(m_flow_nominal=0.1, redeclare
      package Medium = IDEAS.Media.Water.Simple) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={60,16})));
  Annex60.Fluid.Movers.FlowControlled_m_flow fan1(redeclare package Medium =
        Annex60.Media.Water, m_flow_nominal=0.5)
    annotation (Placement(transformation(extent={{-56,6},{-36,26}})));
  Modelica.Blocks.Sources.Pulse pulse(
    width=5,
    period=86400,
    amplitude=0,
    offset=273.15 + 70)
    annotation (Placement(transformation(extent={{-68,30},{-76,38}})));
  Annex60.Fluid.HeatExchangers.HeaterCooler_T    idealHeater1(
    dp_nominal=0,
    redeclare package Medium = Annex60.Media.Water,
    m_flow_nominal=0.1)
    annotation (Placement(transformation(extent={{-82,6},{-62,26}})));
  Modelica.Blocks.Sources.Pulse pulse1(
    period=86400,
    amplitude=0,
    width=0,
    offset=0.1)
    annotation (Placement(transformation(extent={{-64,30},{-56,38}})));
  Buildings.Fluid.FixedResistances.Pipe pip(
    redeclare package Medium = IDEAS.Media.Water.Simple,
    m_flow_nominal=0.1,
    thicknessIns=0.02,
    lambdaIns=0.026,
    length=1000,
    useMultipleHeatPorts=true,
    nSeg=2,
    T_start=273.15 + 70,
    diameter=0.1)
    annotation (Placement(transformation(extent={{16,26},{36,6}})));
  IDEAS.Fluid.Sources.FixedBoundary bou1(
    p=100000,
    T=273.15 + 70,
    redeclare package Medium = IDEAS.Media.Water.Simple,
    nPorts=1) annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=0,
        origin={-94,-26})));
  IDEAS.Fluid.Sources.FixedBoundary bou2(
    use_T=false,
    use_p=true,
    p=100000,
    nPorts=1,
    redeclare package Medium = IDEAS.Media.Water.Simple)
              annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={90,-26})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TMocIn1(
                                                m_flow_nominal=0.1, redeclare
      package Medium = IDEAS.Media.Water.Simple) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-20,-26})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TMocOut1(
                                                 m_flow_nominal=0.1, redeclare
      package Medium = IDEAS.Media.Water.Simple) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={60,-26})));
  Annex60.Fluid.Movers.FlowControlled_m_flow fan2(redeclare package Medium =
        Annex60.Media.Water, m_flow_nominal=0.5)
    annotation (Placement(transformation(extent={{-56,-36},{-36,-16}})));
  Annex60.Fluid.HeatExchangers.HeaterCooler_T    idealHeater2(
    dp_nominal=0,
    redeclare package Medium = Annex60.Media.Water,
    m_flow_nominal=0.1)
    annotation (Placement(transformation(extent={{-82,-36},{-62,-16}})));
  Buildings.Fluid.FixedResistances.Pipe pip1(
    redeclare package Medium = IDEAS.Media.Water.Simple,
    m_flow_nominal=0.1,
    thicknessIns=0.02,
    lambdaIns=0.026,
    nSeg=100,
    length=1000,
    useMultipleHeatPorts=true)
    annotation (Placement(transformation(extent={{16,-36},{36,-16}})));
  Buildings.HeatTransfer.Sources.FixedTemperature fixedTemperature1[100](
                                                                   T=273.15 + 20)
    annotation (Placement(transformation(extent={{-20,-80},{0,-60}})));
equation
  connect(TMocOut.port_b,bou5. ports[1]) annotation (Line(
      points={{70,16},{80,16}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(TMocIn.port_a, fan1.port_b) annotation (Line(
      points={{-30,16},{-36,16}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(bou4.ports[1], idealHeater1.port_a) annotation (Line(
      points={{-90,16},{-82,16}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(idealHeater1.port_b, fan1.port_a) annotation (Line(
      points={{-62,16},{-56,16}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(pulse1.y, fan1.m_flow_in) annotation (Line(
      points={{-55.6,34},{-46.2,34},{-46.2,28}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(pulse.y, idealHeater1.TSet) annotation (Line(
      points={{-76.4,34},{-88,34},{-88,22},{-84,22}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(TMocIn.port_b, pip.port_a) annotation (Line(
      points={{-10,16},{16,16}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(TMocOut.port_a, pip.port_b) annotation (Line(
      points={{50,16},{36,16}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(TMocOut1.port_b, bou2.ports[1]) annotation (Line(
      points={{70,-26},{80,-26}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(TMocIn1.port_a, fan2.port_b) annotation (Line(
      points={{-30,-26},{-36,-26}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(bou1.ports[1],idealHeater2. port_a) annotation (Line(
      points={{-90,-26},{-82,-26}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(idealHeater2.port_b,fan2. port_a) annotation (Line(
      points={{-62,-26},{-56,-26}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(pulse.y,idealHeater2. TSet) annotation (Line(
      points={{-76.4,34},{-88,34},{-88,-20},{-84,-20}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(TMocIn1.port_b, pip1.port_a) annotation (Line(
      points={{-10,-26},{16,-26}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(TMocOut1.port_a, pip1.port_b) annotation (Line(
      points={{50,-26},{36,-26}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(fan2.m_flow_in, fan1.m_flow_in) annotation (Line(
      points={{-46.2,-14},{-46.2,-6},{-52,-6},{-52,34},{-46.2,34},{-46.2,28}},
      color={0,0,127},
      smooth=Smooth.None));

  connect(fixedTemperature.port, pip.heatPorts) annotation (Line(
      points={{0,50},{26,50},{26,21}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(fixedTemperature1.port, pip1.heatPorts) annotation (Line(
      points={{0,-70},{26,-70},{26,-31}},
      color={191,0,0},
      smooth=Smooth.None));
  annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{100,100}}), graphics), Icon(coordinateSystem(extent={{-100,
            -100},{100,100}})),
    experiment(StopTime=604800),
    __Dymola_experimentSetupOutput);
end BuildingTest;
