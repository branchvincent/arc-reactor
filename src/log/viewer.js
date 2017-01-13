var level2name = [];
level2name[0] = "All";
level2name[10] = "Debug",
level2name[20] = "Info";
level2name[30] = "Warn";
level2name[40] = "Error";
level2name[50] = "None";

var level2class = [];
level2class[10] = "",
level2class[20] = "info";
level2class[30] = "warning";
level2class[40] = "danger";

var populate_filter = function(id) {
    var filter = $(id);
    for(var i = 0; i <= 50; i += 10) {
        filter.append($("<option>").attr("value", i).text(level2name[i]));
    }

}

var setup = function() {
    // setup filter dropdown
    populate_filter("#filter");

    $("#reload").click(function() {
        var level = $("#filter option:selected").val();
        clear_all_records("#records");

        var filters = [];
        $("#source_filters").find("select").each(function(i, e) {
            filters[$(e).data("name")] = $(e).children(":selected").val();
        });

        load_all_records("#records", 50, level, filters);
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

var load_records = function(table, skip, count, level, filters, next) {
    s = '';
    for(var key in filters) {
        if(filters.hasOwnProperty(key)) {
            s += key + ":" + filters[key] + "!";
        }
    }
    s = s.substring(0, s.length - 1);

    $.ajax({ url: "records", type: "GET", data: {level: level, skip: skip, count: count, filter: s}, dataType: "json"})
    .done(function(obj){
        var ampm = $("#time_12hr").is(":checked");
        var hostname = $("#host_name").is(":checked");

        var name_update = false;

        var sources = obj.names;
        for(var i = 0; i < sources.length; i++) {
            var id = "filter_" + sources[i].replace('.', '_');

            if($("#" + id).length) {
                continue;
            }
            name_update = true;

            $("<div>").data("name", sources[i]).addClass("form-group").append(
                $("<label>").addClass("col-sm-4 control-label").attr("for", id).text(sources[i].replace(".", " ")),
                $("<div>").addClass("col-sm-8").append(
                    $("<select>").attr("id", id).data("name", sources[i]).addClass("form_control")
                )
            ).appendTo("#source_filters");

            populate_filter("#" + id);
        }

        // sort divs after adding filter entries
        if(name_update) {
            $("#source_filters").children("div").sort(function(a, b) {
                return $(a).data("name") > $(b).data("name");
            }).appendTo("#source_filters");
        }

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

        var percent = 100.0 * (skip + obj.records.length) / obj.available;

        $("#progress").css("width", percent + "%");

        next(table, skip, obj.records.length);
    });
}

var load_all_records = function(table, rate, level, filters) {
    var next = function(table, skip, count) {
        if(count < rate) {
            // done
        } else {
            load_records(table, skip + rate, rate, level, filters, next)
        }
    };

    load_records("#records", 0, rate, level, filters, next);
}

var clear_all_records = function(table) {
    $(table + " > tbody").empty();
}

$(document).ready(function() {
    setup();
    load_all_records("#records", 50, 0);
});