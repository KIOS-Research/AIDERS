function find_difference_on_two_arrays(a1, a2)
{
    var a = [], diff = [];

    for (var i = 0; i < a1.length; i++)
    {
        a[a1[i]] = true;
    }

    for (var i = 0; i < a2.length; i++)
    {
        if (a[a2[i]])
        {
            delete a[a2[i]];
        }
        else
        {
            a[a2[i]] = true;
        }
    }

    for (var k in a)
    {
        diff.push(k);
    }

    return diff;
}


/*Checks if given object is empty or not*/
function isObjEmpty(obj)
{
    for (var key in obj)
    {
        if (obj.hasOwnProperty(key))
        {
            return false;
        }
    }
    return true;
}


function hexToRgb(hex)
{
    var result = /^#?([a-f\d]{2})([a-f\d]{2})([a-f\d]{2})$/i.exec(hex);
    return [parseInt(result[1], 16), parseInt(result[2], 16), parseInt(result[3], 16)]
}


function randomNumber(min, max) {
    return Math.random() * (max - min) + min;
}


function onlyUnique(value, index, self) {
    return self.indexOf(value) === index;
}

function get_last_part_of_url(str)
{
    return str.substring(str.lastIndexOf('/') + 1)
}

function remove_duplicates(arr)
{
    return arr.filter(onlyUnique);
}


/*
* Returns the difference between the specified arrays that contain objects
* */
function subtract_two_object_arrays(arrayA, arrayB)
{
    let newArr = []

    if (arrayB.length === 0)
    {
        return arrayA
    }
    for (let i = 0; i < arrayA.length; i++)
    {

        let theSame = false;
        for (let j=0; j<arrayB.length; j++)
        {
            if (JSON.stringify(arrayA[i]) === JSON.stringify(arrayB[j]))
            {
                theSame = true
                break
            }
        }

        if (!theSame)
        {
            newArr.push(arrayA[i])
        }
    }

    return newArr
}

function expandedLog(item, maxDepth = 100, depth = 0){
    if (depth > maxDepth ) {
        console.log(item);
        return;
    }
    if (typeof item === 'object' && item !== null) {
        Object.entries(item).forEach(([key, value]) => {
            console.group(key + ' : ' +(typeof value));
            expandedLog(value, maxDepth, depth + 1);
            console.groupEnd();
        });
    } else {
        console.log(item);
    }
}



var removeByAttr = function(arr, attr, value){
    var i = arr.length;
    while(i--){
        if( arr[i]
            && arr[i].hasOwnProperty(attr)
            && (arguments.length > 2 && arr[i][attr] === value ) ){

            arr.splice(i,1);

        }
    }
    return arr;
}

// function convertUTCDateToLocalDate(datetime) {
//     console.log("datetime1: ", datetime)
//         datetime = new Date(datetime.concat(' UTC'));
//         console.log("datetime2: ", datetime)
//
//         datetime.toString()
//             console.log("datetime3: ", datetime)
//     console.log("datetime4: ",  datetime.toString().substring(0, datetime.toString().indexOf(' GMT')))
//         return datetime.toString().substring(0, datetime.toString().indexOf(' GMT'))
//
// }

function convertUTCDateToLocalDate(date) {
  var dateLocal = new Date(date);
  var newDate = new Date(dateLocal.getTime() - dateLocal.getTimezoneOffset()*60*1000);
  return newDate.toString().substring(0, newDate.toString().indexOf(' GMT'))
}
const metersToPixelsAtMaxZoom = (meters, latitude) =>
    meters / 0.075 / Math.cos(latitude * Math.PI / 180)