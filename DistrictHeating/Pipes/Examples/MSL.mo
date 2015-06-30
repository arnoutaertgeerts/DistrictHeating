within DistrictHeating.Pipes.Examples;
model MSL
  import DistrictHeating;
  extends Modelica.Icons.Example;
  import IDEAS;









  Buildings.HeatTransfer.Sources.FixedTemperature fixedTemperature(T=273.15 + 20)
    annotation (Placement(transformation(extent={{-86,70},{-66,90}})));
  IDEAS.Fluid.Sources.FixedBoundary bou4(
    nPorts=1,
    p=100000,
    T=273.15 + 70,
    redeclare package Medium = IDEAS.Media.Water.Simple)
              annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-86,-10})));
  IDEAS.Fluid.Sources.FixedBoundary bou5(
    use_T=false,
    use_p=true,
    p=100000,
    nPorts=1,
    redeclare package Medium = IDEAS.Media.Water.Simple)
              annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={90,-10})));
  IDEAS.Fluid.Movers.Pump pump1(redeclare package Medium =
        IDEAS.Media.Water.Simple, m_flow_nominal=0.1,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    energyDynamics=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-60,-10})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TMocIn(m_flow_nominal=0.1, redeclare
      package Medium = IDEAS.Media.Water.Simple) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-32,-10})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TMocOut(m_flow_nominal=0.1, redeclare
      package Medium = IDEAS.Media.Water.Simple) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={60,-10})));
  Modelica.Fluid.Pipes.DynamicPipe MSL(
    use_HeatTransfer=true,
    nNodes=50,
    redeclare package Medium = Annex60.Media.Water,
    T_start=273.15 + 20,
    length=100,
    diameter=0.05,
    redeclare model HeatTransfer =
        Modelica.Fluid.Pipes.BaseClasses.HeatTransfer.ConstantFlowHeatTransfer
        (T_ambient=273.15 + 20, alpha0=0.026*18.6737/(Modelica.Constants.pi*
            0.05)))
    annotation (Placement(transformation(extent={{-10,-20},{10,0}})));
  Modelica.Thermal.HeatTransfer.Components.ThermalCollector colAllToOne(m=50)
    "Connector to assign multiple heat ports to one heat port" annotation (
      Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=180,
        origin={0,2})));
equation
  connect(pump1.port_b,TMocIn. port_a) annotation (Line(
      points={{-50,-10},{-42,-10}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(TMocOut.port_b,bou5. ports[1]) annotation (Line(
      points={{70,-10},{80,-10}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(bou4.ports[1],pump1. port_a) annotation (Line(
      points={{-76,-10},{-70,-10}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(TMocIn.port_b,MSL. port_a) annotation (Line(
      points={{-22,-10},{-10,-10}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(MSL.port_b,TMocOut. port_a) annotation (Line(
      points={{10,-10},{50,-10}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(colAllToOne.port_a,MSL. heatPorts) annotation (Line(
      points={{0,-4},{0,-5.6},{0.1,-5.6}},
      color={191,0,0},
      smooth=Smooth.None));
  connect(fixedTemperature.port, colAllToOne.port_b) annotation (Line(
      points={{-66,80},{0,80},{0,8}},
      color={191,0,0},
      smooth=Smooth.None));
  annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{100,100}}), graphics), Icon(coordinateSystem(extent={{-100,
            -100},{100,100}})),
    experiment(StopTime=25000),
    __Dymola_experimentSetupOutput);
end MSL;
