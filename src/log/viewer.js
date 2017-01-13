var level2name = [];
level2name[0] = "None";
level2name[10] = "Debug",
level2name[20] = "Info";
level2name[30] = "Warn";
level2name[40] = "Error";

var level2class = [];
level2class[10] = "",
level2class[20] = "info";
level2class[30] = "warning";
level2class[40] = "danger";

var setup = function() {
    // setup filter dropdown
    var filter = $("#filter");
    for(var i = 0; i <= 40; i += 10) {
        filter.append($("<option>").attr("value", i).text(level2name[i]));
    }

    $("#reload").click(function() {
        var level = $("#filter option:selected").val();
        clear_all_records("#records");
        load_all_records("#records", 50, level);
    });
}

var format_date = function(millis, ampm) {
    var date = new Date(millis);

    var s = (date.getMonth() + 1) + "/" + date.getDate() + "/" + date.getFullYear() + "&nbsp;";
    if(ampm) {
        s += (date.getHours() % 12) || 12;
    } else {
        s += date.getHours();
    }
    s += ":" + date.getMinutes() + ":" + date.getSeconds() + "." + date.getMilliseconds();
    if(ampm) {
        s += "&nbsp;" + (date.getHours() < 12 ? "am" : "pm");
    }

    return s;
}

var load_records = function(table, skip, count, level, next) {
    $.ajax({ url: "records", type: "GET", data: {level: level, skip: skip, count: count}, dataType: "json"})
    .done(function(obj){
        var ampm = $("#date_12hr").is(":checked");

        for(var i = 0; i < obj.records.length; i++) {
            var record = obj.records[i];

            var icon = $("<span>");
            if(record.levelno >= 40) {
                icon.addClass("glyphicon glyphicon-exclamation-sign");
            //} else if(record.levelno >= 30) {
            //    icon.addClass("glyphicon glyphicon-alert");
            }

            var lines = record.message.split(/(?:\n|\r)+/g);
            var message = $("<td>").append(lines[0].trim());
            for(var j = 1; j < lines.length; j++) {
                message.append("<br />").append(lines[j].trim());
            }

            if(record.exception) {
                message
                .append("&nbsp;")
                .append($("<a>")
                        .attr("role", "button")
                        .attr("data-toggle", "collapse")
                        .attr("data-target", "#extra" + record.id)
                        .attr("href", "#record" + record.id)
                        .text(">>>"))
                .append($("<pre>")
                    .attr("id", "extra" + record.id)
                    .addClass("collapse")
                    .text(record.exception));
            }

            $("<tr>").attr("id", "record" + record.id).append(
                $("<td>").append(format_date(1000 * record.created, ampm)),
                $("<td>").text((hostname ? record.hostname : record.ip) + ":" + record.port),
                $("<td>").text(record.name),
                $("<td>").text(record.pathname + ":" + record.lineno),
                $("<td>").append(icon),
                message
            ).addClass(level2class[record.levelno]).appendTo("#records");
        }

        var n = skip + obj.records.length;

        $("#progress").css("width", 100.0 * n / obj.available + "%");

        next(table, skip, obj.records.length);
    });
}

var load_all_records = function(table, rate, level) {
    var next = function(table, skip, count) {
        if(count < rate) {
            $("#progress").hide();
        } else {
            load_records(table, skip + rate, rate, level, next)
        }
    };

    $("#progress").css("width", 0).show();

    load_records("#records", 0, rate, level, next);
}

var clear_all_records = function(table) {
    console.log('clear ' + table);
    $(table + " > tbody").empty();
}

$(document).ready(function() {
    setup();
    load_all_records("#records", 50, 0);
});