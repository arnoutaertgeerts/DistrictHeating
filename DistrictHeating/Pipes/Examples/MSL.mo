within DistrictHeating.Pipes.Examples;
model MSL
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
  Modelica.Fluid.Pipes.DynamicPipe MSL(
    use_HeatTransfer=true,
    redeclare package Medium = Annex60.Media.Water,
    T_start=273.15 + 20,
    length=50,
    diameter=0.05,
    redeclare model HeatTransfer =
        Modelica.Fluid.Pipes.BaseClasses.HeatTransfer.ConstantFlowHeatTransfer
        (T_ambient=273.15 + 20, alpha0=0.026*18.6373/(Modelica.Constants.pi*
            0.05)),
    nNodes=20)
    annotation (Placement(transformation(extent={{0,-40},{20,-20}})));
  Modelica.Thermal.HeatTransfer.Components.ThermalCollector colAllToOne(m=MSL.nNodes)
    "Connector to assign multiple heat ports to one heat port" annotation (
      Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=180,
        origin={10,-16})));
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
  Modelica.Fluid.Pipes.DynamicPipe MSL1(
    use_HeatTransfer=true,
    redeclare package Medium = Annex60.Media.Water,
    T_start=273.15 + 20,
    length=50,
    diameter=0.05,
    redeclare model HeatTransfer =
        Modelica.Fluid.Pipes.BaseClasses.HeatTransfer.ConstantFlowHeatTransfer
        (T_ambient=273.15 + 20, alpha0=0.026*18.6373/(Modelica.Constants.pi*
            0.05)),
    nNodes=MSL.nNodes)
    annotation (Placement(transformation(extent={{26,-40},{46,-20}})));
  Modelica.Thermal.HeatTransfer.Components.ThermalCollector colAllToOne1(m=MSL.nNodes)
    "Connector to assign multiple heat ports to one heat port" annotation (
      Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=180,
        origin={36,-16})));
equation
  connect(TMocOut.port_b,bou5. ports[1]) annotation (Line(
      points={{70,-30},{80,-30}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(TMocIn.port_b,MSL. port_a) annotation (Line(
      points={{-10,-30},{0,-30}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(colAllToOne.port_a,MSL. heatPorts) annotation (Line(
      points={{10,-22},{10,-25.6},{10.1,-25.6}},
      color={191,0,0},
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
  connect(MSL.port_b, MSL1.port_a) annotation (Line(
      points={{20,-30},{26,-30}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(MSL1.port_b, TMocOut.port_a) annotation (Line(
      points={{46,-30},{50,-30}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(MSL1.heatPorts, colAllToOne1.port_a) annotation (Line(
      points={{36.1,-25.6},{36,-25.6},{36,-22}},
      color={127,0,0},
      smooth=Smooth.None));
  connect(fixedTemperature.port, colAllToOne.port_b) annotation (Line(
      points={{0,10},{10,10},{10,-10}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(fixedTemperature.port, colAllToOne1.port_b) annotation (Line(
      points={{0,10},{36,10},{36,-10}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(pulse.y, idealHeater1.TSet) annotation (Line(
      points={{-76.4,-12},{-88,-12},{-88,-24},{-84,-24}},
      color={0,0,127},
      smooth=Smooth.None));
  annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{100,100}}), graphics), Icon(coordinateSystem(extent={{-100,
            -100},{100,100}})),
    experiment(StopTime=604800),
    __Dymola_experimentSetupOutput);
end MSL;
