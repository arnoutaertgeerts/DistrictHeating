within DistrictHeating.Pipes.Examples;
model PreinsulatedPipesExample

  //Extensions
  extends Modelica.Icons.Example;

  Modelica.Fluid.Sources.FixedBoundary returnsink(
    nPorts=1,
    redeclare package Medium =
        Modelica.Media.Water.ConstantPropertyLiquidWater,
    use_T=false,
    T=343.15)
    annotation (Placement(transformation(extent={{-94,38},{-74,58}})));

  Modelica.Fluid.Sources.MassFlowSource_T boundary(
    redeclare package Medium =
        Modelica.Media.Water.ConstantPropertyLiquidWater,
    m_flow=0.2,
    nPorts=1,
    T=343.15) annotation (Placement(transformation(extent={{-92,70},{-72,90}})));

  Modelica.Fluid.Sources.MassFlowSource_T boundary1(
    redeclare package Medium =
        Modelica.Media.Water.ConstantPropertyLiquidWater,
    m_flow=0.2,
    nPorts=1,
    T=318.15) annotation (Placement(transformation(extent={{92,38},{72,58}})));

  Modelica.Fluid.Sources.FixedBoundary supplysink(
    redeclare package Medium =
        Modelica.Media.Water.ConstantPropertyLiquidWater,
    use_T=false,
    T=343.15,
    nPorts=1) annotation (Placement(transformation(extent={{90,70},{70,90}})));

  IDEAS.Fluid.Sensors.TemperatureTwoPort TReturn(redeclare package Medium =
        Modelica.Media.Water.ConstantPropertyLiquidWater, m_flow_nominal=0.1)
    annotation (Placement(transformation(extent={{-40,38},{-60,58}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TSupply(redeclare package Medium =
        Modelica.Media.Water.ConstantPropertyLiquidWater, m_flow_nominal=0.1)
    annotation (Placement(transformation(extent={{16,70},{36,90}})));
  Modelica.Blocks.Sources.Constant TGround(k=273 + 12)
    annotation (Placement(transformation(extent={{-70,0},{-50,20}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TSupply1(redeclare package Medium =
        Modelica.Media.Water.ConstantPropertyLiquidWater, m_flow_nominal=0.1)
    annotation (Placement(transformation(extent={{26,-22},{46,-2}})));
  DoublePipes.TwinPipeGround twinPipeGround(
    redeclare package Medium1 =
        Modelica.Media.Water.ConstantPropertyLiquidWater,
    redeclare package Medium2 =
        Modelica.Media.Water.ConstantPropertyLiquidWater,
    L=100,
    rho=1000,
    lambdaG=1.5,
    lambdaI=0.026,
    H=1.5,
    E=0.35,
    Do=0.16,
    Di=0.16,
    dp_nominal=20,
    Dc=0.5,
    m1_flow_nominal=0.1,
    m2_flow_nominal=0.1)
    annotation (Placement(transformation(extent={{-12,-52},{8,-24}})));
  DoublePipes.PreinsulatedPipes preinsulatedPipes(
    redeclare package Medium1 =
        Modelica.Media.Water.ConstantPropertyLiquidWater,
    redeclare package Medium2 =
        Modelica.Media.Water.ConstantPropertyLiquidWater,
    L=100,
    rho=1000,
    lambdaG=1.5,
    lambdaI=0.026,
    H=1.5,
    E=0.35,
    dp_nominal=20,
    m1_flow_nominal=0.1,
    m2_flow_nominal=0.1,
    Do=0.25,
    Di=0.16) annotation (Placement(transformation(extent={{-16,46},{4,74}})));
  Production.PolynomialProduction polynomialProduction(
    QNom=10000,
    m_flow_nominal=0.5,
    redeclare package Medium = Modelica.Media.Water.ConstantPropertyLiquidWater,
    redeclare Production.Data.Polynomials.Boiler2ndDegree
      data) annotation (Placement(transformation(extent={{-98,-28},{-78,-50}})));

  IDEAS.Fluid.Sensors.TemperatureTwoPort TReturn1(redeclare package Medium =
        Modelica.Media.Water.ConstantPropertyLiquidWater, m_flow_nominal=0.1)
    annotation (Placement(transformation(extent={{46,-78},{26,-58}})));
  IDEAS.Fluid.FixedResistances.Pipe_HeatPort pipe_HeatPort(redeclare package
      Medium = Modelica.Media.Water.ConstantPropertyLiquidWater, m_flow_nominal=
       0.5) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={62,-42})));
  Modelica.Fluid.Sources.FixedBoundary supplysink1(
    redeclare package Medium =
        Modelica.Media.Water.ConstantPropertyLiquidWater,
    use_T=false,
    nPorts=1,
    T=343.15) annotation (Placement(transformation(extent={{10,-10},{-10,10}},
        rotation=90,
        origin={18,12})));
  Modelica.Thermal.HeatTransfer.Sources.FixedHeatFlow prescribedHeatFlow(Q_flow=
       -8000)
    annotation (Placement(transformation(extent={{104,-32},{84,-10}})));
  IDEAS.Fluid.Movers.Pump pump(m_flow_nominal=0.5, redeclare package Medium =
        Modelica.Media.Water.ConstantPropertyLiquidWater)
    annotation (Placement(transformation(extent={{-64,-30},{-44,-10}})));
  Modelica.Blocks.Sources.Constant TSet(k=273 + 70)
    annotation (Placement(transformation(extent={{-114,-88},{-94,-68}})));
equation
  connect(returnsink.ports[1], TReturn.port_b) annotation (Line(
      points={{-74,48},{-60,48}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(TSupply.port_b, supplysink.ports[1]) annotation (Line(
      points={{36,80},{70,80}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(TSupply1.port_a, twinPipeGround.port_b1) annotation (Line(
      points={{26,-12},{12,-12},{12,-32},{8,-32}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(boundary.ports[1], preinsulatedPipes.port_a1) annotation (Line(
      points={{-72,80},{-30,80},{-30,66},{-16,66}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(preinsulatedPipes.port_b1, TSupply.port_a) annotation (Line(
      points={{4,66},{10,66},{10,80},{16,80}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(TReturn.port_a, preinsulatedPipes.port_b2) annotation (Line(
      points={{-40,48},{-26,48},{-26,54},{-16,54}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(preinsulatedPipes.port_a2, boundary1.ports[1]) annotation (Line(
      points={{4,54},{12,54},{12,48},{72,48}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(TGround.y, preinsulatedPipes.Tg) annotation (Line(
      points={{-49,10},{-6,10},{-6,45.8}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(TGround.y, twinPipeGround.Tg) annotation (Line(
      points={{-49,10},{-30,10},{-30,-64},{-2,-64},{-2,-52.2}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(twinPipeGround.port_a2, TReturn1.port_b) annotation (Line(
      points={{8,-44},{14,-44},{14,-68},{26,-68}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(TSupply1.port_b, pipe_HeatPort.port_a) annotation (Line(
      points={{46,-12},{62,-12},{62,-32}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(pipe_HeatPort.port_b, TReturn1.port_a) annotation (Line(
      points={{62,-52},{62,-68},{46,-68}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(polynomialProduction.port_a, twinPipeGround.port_b2) annotation (Line(
      points={{-77.8,-42},{-78,-42},{-78,-56},{-20,-56},{-20,-44},{-12,-44}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(supplysink1.ports[1], twinPipeGround.port_b1) annotation (Line(
      points={{18,2},{18,-12},{12,-12},{12,-32},{8,-32}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(pipe_HeatPort.heatPort, prescribedHeatFlow.port) annotation (Line(
      points={{72,-42},{80,-42},{80,-21},{84,-21}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(polynomialProduction.port_b, pump.port_a) annotation (Line(
      points={{-77.8,-34},{-78,-34},{-78,-20},{-64,-20}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(pump.port_b, twinPipeGround.port_a1) annotation (Line(
      points={{-44,-20},{-20,-20},{-20,-32},{-12,-32}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(TSet.y, polynomialProduction.TSet) annotation (Line(
      points={{-93,-78},{-90,-78},{-90,-50},{-89,-50}},
      color={0,0,127},
      smooth=Smooth.None));
  annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{100,100}}), graphics));
end PreinsulatedPipesExample;
