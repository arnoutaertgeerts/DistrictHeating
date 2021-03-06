within DistrictHeating.HeatingSystems.Control;
model MassFlowControl
  extends PartialHXControl;

  IDEAS.Controls.Continuous.LimPID conPID
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-40,0})));
equation
  connect(senMassFlow2, conPID.u_s) annotation (Line(
      points={{104,40},{-20,40},{-20,-1.33227e-015},{-28,-1.33227e-015}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(conPID.y, y) annotation (Line(
      points={{-51,1.33227e-015},{-48,1.33227e-015},{-48,0},{-106,0}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(conPID.u_m, senMassFlow1) annotation (Line(
      points={{-40,12},{-40,22},{32,22},{32,-80},{104,-80}},
      color={0,0,127},
      smooth=Smooth.None));
  annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{100,100}}),      graphics), Icon(coordinateSystem(
          preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
        graphics={              Rectangle(
        extent={{-100,-100},{100,100}},
        lineColor={0,0,127},
        fillColor={255,255,255},
        fillPattern=FillPattern.Solid),
        Line(points={{-80,78},{-80,-90}}, color={192,192,192}),
        Polygon(
          points={{-80,90},{-88,68},{-72,68},{-80,90}},
          lineColor={192,192,192},
          fillColor={192,192,192},
          fillPattern=FillPattern.Solid),
        Line(points={{-90,-80},{82,-80}}, color={192,192,192}),
        Polygon(
          points={{90,-80},{68,-72},{68,-88},{90,-80}},
          lineColor={192,192,192},
          fillColor={192,192,192},
          fillPattern=FillPattern.Solid),
        Line(points={{-80,-80},{-80,-20},{30,60},{80,60}}, color={0,0,127}),
        Rectangle(
          extent={{-6,-20},{66,-66}},
          lineColor={255,255,255},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Text(
          extent={{-40,22},{58,-62}},
          lineColor={0,128,255},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          textString="M")}));
end MassFlowControl;
