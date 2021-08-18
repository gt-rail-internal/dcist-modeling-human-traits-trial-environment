// admin.js: functions for tabulating user data

function updateTable() {
    // get the latest participant data
    var data = {
        "1854": {
            "sa": .3,
            "ni": .4,
            "ot": .5,
            "s1": .45,
            "s2": .56,
            "s3": .95,
        },
        "3948": {
            "sa": .42,
            "ni": .25,
            "ot": .94,
            "s1": .29,
            "s2": .49,
            "s3": .53,
        },
    }

    //  create a table row for each participant
    let table = document.getElementById("score-table");
    table.innerHTML = "";

    // create the header row
    let row = document.createElement("tr");
    row.className = "admin-table";

    // create the row elements
    let pid = document.createElement("td");
    pid.innerHTML = "<b>Participant</b>";
    let sa = document.createElement("td");
    sa.innerHTML = "<b>SA Test</b>";
    let ni = document.createElement("td");
    ni.innerHTML = "<b>NI Test</b>";
    let ot = document.createElement("td");
    ot.innerHTML = "<b>OT Test</b>";
    let s1 = document.createElement("td");
    s1.innerHTML = "<b>Stage 1</b>";
    let s2 = document.createElement("td");
    s2.innerHTML = "<b>Stage 2</b>";
    let s3 = document.createElement("td");
    s3.innerHTML = "<b>Stage 3</b>";

    pid.className = "admin-table";
    sa.className = "admin-table";
    ni.className = "admin-table";
    ot.className = "admin-table";
    s1.className = "admin-table";
    s2.className = "admin-table";
    s3.className = "admin-table";

    // add the elements to the row
    row.appendChild(pid);
    row.appendChild(sa);
    row.appendChild(ni);
    row.appendChild(ot);
    row.appendChild(s1);
    row.appendChild(s2);
    row.appendChild(s3);

    // add the row to the table
    table.appendChild(row);

    data = fetch("/process-data").then((response) => response.json()).then((data) => {
        console.log("received data", data);
        // add the data to the table
        for (i in data) {
            // create the row
            let row = document.createElement("tr");
            row.className = "admin-table";

            // create the row elements
            let pid = document.createElement("td");
            pid.innerHTML = i;
            let sa = document.createElement("td");
            sa.innerHTML = data[i]["sa"];
            let ni = document.createElement("td");
            ni.innerHTML = data[i]["ni"];
            let ot = document.createElement("td");
            ot.innerHTML = data[i]["ot"];
            let s1 = document.createElement("td");
            s1.innerHTML = data[i]["s1"];
            let s2 = document.createElement("td");
            s2.innerHTML = data[i]["s2"];
            let s3 = document.createElement("td");
            s3.innerHTML = data[i]["s3"];

            pid.className = "admin-table";
            sa.className = "admin-table";
            ni.className = "admin-table";
            ot.className = "admin-table";
            s1.className = "admin-table";
            s2.className = "admin-table";
            s3.className = "admin-table";

            // add the elements to the row
            row.appendChild(pid);
            row.appendChild(sa);
            row.appendChild(ni);
            row.appendChild(ot);
            row.appendChild(s1);
            row.appendChild(s2);
            row.appendChild(s3);

            // add the row to the table
            table.appendChild(row);
        }
    });
}