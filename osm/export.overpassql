[bbox:54.8, 82.6, 55.2, 83.2];
(
  (
    (way [highway~"^(motorway|motorway_link|trunk|trunk_link\
            |primary|primary_link|secondary|secondary_link\
            |tertiary|tertiary_link|unclassified\
            |residential|residential_link)$"]
      ({{bbox}});
     >;
   );
    rel[type=restriction](bw);
    <<;
  );
  //>;
)->.roads;

(
  way[building]
  ({{bbox}});
  >>;
)->.buildings;

(
  .roads;
  .buildings;
);

out body;
