A* ROUTE PLANNER - QUICK START

FILES
- route_planner.py: complete Python implementation
- requirements.txt: dependencies
- results.csv: 30-route run results
- results_summary.json: average metrics
- route_map.html: interactive route map from the offline demo run
- Project_Report_8_Pages.docx: concise report

RUN ON YOUR COMPUTER (REAL OPENSTREETMAP DATA)
1. Install Python 3.10 or newer.
2. Open Terminal/Command Prompt in this folder.
3. Run: pip install -r requirements.txt
4. Run: python route_planner.py --place "India Gate, New Delhi, India" --radius 2500
5. Wait for the OpenStreetMap download. The program creates results.csv, results_summary.json and route_map.html.
6. Open route_map.html in a browser.

OFFLINE TEST
python route_planner.py --offline
This creates a reproducible synthetic grid for testing. It is NOT real OpenStreetMap data.

CHANGE LOCATION
Replace the --place value with a city landmark or address. Increase/decrease --radius in metres.

METHOD
A* uses f(n)=g(n)+h(n), where g is road distance travelled and h is Haversine straight-line distance to the destination. Dijkstra uses f(n)=g(n) and is the optimal reference. The script compares 30 reproducible start/goal pairs.

NOTE
The report's current numerical results are from the offline synthetic graph because OSMnx could not be installed in the generation environment. For a submission that explicitly requires a real road network, run the real OpenStreetMap command above and replace the offline results in the report with the newly generated results.
