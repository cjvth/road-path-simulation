[bbox:54.8, 82.6, 55.2, 83.2];
(
  (
    (way [highway~"^(motorway|motorway_link|trunk|trunk_link|primary|primary_link|secondary|secondary_link|tertiary|tertiary_link|unclassified|residential|residential_link|unclassified)$"]
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
  (
    (way [highway~"^(motorway|motorway_link|trunk|trunk_link|primary|primary_link|secondary|secondary_link|tertiary|tertiary_link|unclassified|residential|residential_link|unclassified|construction|service)$"]
      (54.991922, 82.860172, 55.019055,  82.934172);
     >;
   );
    rel[type=restriction](bw);
    <<;
  );
  //>;
)->.bridge;


(
  .roads;
  .buildings;
  .bridge;
);

out body;

