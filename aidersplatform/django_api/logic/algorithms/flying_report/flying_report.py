from reportlab.platypus import (BaseDocTemplate, Flowable, Frame, PageBreak,
                                PageTemplate, Paragraph, Spacer, Table,
                                TableStyle)
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.pagesizes import A4
from reportlab.lib.enums import TA_CENTER, TA_JUSTIFY, TA_LEFT, TA_RIGHT
from django.core.files.storage import default_storage
import requests
import pdfkit
import numpy as np
import json
import os
import re
import xml.etree.ElementTree as ET
from datetime import datetime, timedelta
from os import path


styleSheet = getSampleStyleSheet()
page_width, page_height = A4


courier_text = ParagraphStyle(name='text',
                              fontName='Courier-Bold',
                              fontSize=8)


class PdfImage(Flowable):
    """
    PdfImage wraps the first page from a PDF file as a Flowable
    which can be included into a ReportLab Platypus document.
    Based on the vectorpdf extension in rst2pdf (http://code.google.com/p/rst2pdf/)

    This can be used from the place where you want to return your matplotlib image
    as a Flowable:

    img = BytesIO()

    fig, ax = plt.subplots(figsize=(canvaswidth,canvaswidth))

    ax.plot([1,2,3],[6,5,4],antialiased=True,linewidth=2,color='red',label='a curve')

    fig.savefig(img,format='PDF')

    return(PdfImage(img))

    """

    def __init__(self, filename_or_object, width=None, height=None, kind='direct', coordinates=None):
        # If using StringIO buffer, set pointer to begining
        if hasattr(filename_or_object, 'read'):
            filename_or_object.seek(0)
            # print("read")
        xrange = range
        from pdfrw import PdfReader
        from pdfrw.buildxobj import pagexobj
        self.page = PdfReader(filename_or_object, decompress=False).pages[0]
        self.xobj = pagexobj(self.page)

        self.imageWidth = width
        self.imageHeight = height
#        if coordinates == None:
#            x1, y1, x2, y2 = self.xobj.BBox
#        else:
#            x1, y1, x2, y2 = coordinates
        x1, y1, x2, y2 = self.xobj.BBox
        # print('xxxx: ', x1, y1, x2, y2)

        self._w, self._h = x2 - x1, y2 - y1
        if not self.imageWidth:
            self.imageWidth = self._w
        if not self.imageHeight:
            self.imageHeight = self._h
        self.__ratio = float(self.imageWidth)/self.imageHeight
        if kind in ['direct', 'absolute'] or width == None or height == None:
            self.drawWidth = width or self.imageWidth
            self.drawHeight = height or self.imageHeight
        elif kind in ['bound', 'proportional']:
            factor = min(float(width)/self._w, float(height)/self._h)
            self.drawWidth = self._w*factor
            self.drawHeight = self._h*factor

    def wrap(self, availableWidth, availableHeight):
        """
        returns draw- width and height

        convenience function to adapt your image
        to the available Space that is available
        """
        return self.drawWidth, self.drawHeight

    def drawOn(self, canv, x, y, _sW=0):
        """
        translates Bounding Box and scales the given canvas
        """
        if _sW > 0 and hasattr(self, 'hAlign'):
            a = self.hAlign
            if a in ('CENTER', 'CENTRE', TA_CENTER):
                x += 0.5*_sW
            elif a in ('RIGHT', TA_RIGHT):
                x += _sW
            elif a not in ('LEFT', TA_LEFT):
                raise ValueError("Bad hAlign value " + str(a))

        #xobj_name = makerl(canv._doc, self.xobj)
        from pdfrw.toreportlab import makerl
        xobj_name = makerl(canv, self.xobj)

        xscale = self.drawWidth/self._w
        yscale = self.drawHeight/self._h

        x -= self.xobj.BBox[0] * xscale
        y -= self.xobj.BBox[1] * yscale

        canv.saveState()
        canv.translate(x, y)
        canv.scale(xscale, yscale)
        canv.doForm(xobj_name)
        canv.restoreState()


def _wx2pdf(website, filename='temporary.pdf'):
    pdfkit.from_url(website, filename)
    return filename


def calculate_charge(drone=1, time=30):
    mission = [30, 30]
    contingency = [5, 5]
    final = [20, 20]
    return mission, contingency, final


def _operations(story, d):

    text_head = '''
    KIOS UAV OFP 1, OPERATION '''+d['operation']+''', DATE/TIME CREATED {} <br />
    OPERATOR REGISTRATION NAME: '''.format(
        datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    )+d['pilot_id']+''' <br />
    '''
    story.append(Paragraph(text_head, courier_text))
    story.append(Spacer(1, 8))
    text_flight_operations_details = '''
        FLIGHT/OPERATIONS DETAILS <br />
        ========================= <br />
        COORDINATE OF OPERATION: {}<br />
        ALTITUDE OF OPERATION: {}<br />
        RADIUS OF OPERATION: {}<br />
        BUFFER ALTITUDE OF OPERATION: {}<br />
        BUFFER RADIUS OF OPERATION: {}<br />
        OPERATIONS START: {} <br />
        OPERATIONS END: {} <br />
        '''.format(
        str(d['lat']) + 'N ' + str(d['lon']) + 'E',
        d['max_altitude'],
        d['operational_radius'],
        d['buffer_radius'],
        d['buffer_altitude'],
        d['op_start'].strftime("%H:%M"),  # d['datetime_now'] #
        d['op_end'].strftime("%H:%M"),
    )

    story.append(Paragraph(text_flight_operations_details, courier_text))
    # story.append(Spacer(1, 8))


#    text_drone_details = '''
#    DRONE DETAILS <br />
#    ============= <br />
#    MAKE: {}<br />
#    MODEL: {}<br />
#    KIOS IDENTIFIER: {}<br />
#    SERIAL NUMBER: {}<br />
#    REGISTRATION ID: {}<br />
#    DRONE CLASS: {}<br />
#    '''.format(
#        d['make'],
#        d['model'],
#        d['kios_identifier'],
#        d['serial_number'],
#        d['registration_number'],
#        d['drone_class']
#        )
#
##    story.append( Paragraph( text_drone_details, courier_text) )
##    story.append(Spacer(1, 8))
#
#    text_weights = '''
#    WEIGHTS <br />
#    ======= <br />
#    DRONE MASS: {} grams <br />
#    MAX PAYLOAD: {} grams <br />
#    MTOM: {} grams <br />
#    '''.format(
#        d['mass'],
#        d['mtom'] - d['mass'],
#        d['mtom']
#        )


#    t = Table( [( Paragraph( text_drone_details, courier_text), Paragraph( text_weights, courier_text) )], colWidths=[200, 330] )
#    t.setStyle(TableStyle([('VALIGN',(0,0),(1,0),'TOP')]))
#    story.append(t)
#    story.append(Spacer(1, 8))

    # for drone_id in range(len(d['drone_info'])):

    #     text_drone_details = '''
    #     DRONE DETAILS <br />
    #     ============= <br />
    #     MAKE: {}<br />
    #     MODEL: {}<br />
    #     KIOS IDENTIFIER: {}<br />
    #     SERIAL NUMBER: {}<br />
    #     REGISTRATION ID: {}<br />
    #     DRONE CLASS: {}<br />
    #     '''.format(
    #         d['drone_info'][drone_id]['make'],
    #         d['drone_info'][drone_id]['model'],
    #         d['drone_info'][drone_id]['kios_identifier'],
    #         d['drone_info'][drone_id]['serial_number'],
    #         d['drone_info'][drone_id]['registration_number'],
    #         d['drone_info'][drone_id]['drone_class']
    #     )

    #     text_weights = '''
    #         WEIGHTS <br />
    #         ======= <br />
    #         DRONE MASS: {} grams <br />
    #         MAX PAYLOAD: {} grams <br />
    #         MTOM: {} grams <br />
    #         '''.format(
    #         d['drone_info'][drone_id]['mass'],
    #         d['drone_info'][drone_id]['mtom'] -
    #         d['drone_info'][drone_id]['mass'],
    #         d['drone_info'][drone_id]['mtom']
    #     )

    #     t = Table([(Paragraph(text_drone_details, courier_text), Paragraph(
    #         text_weights, courier_text))], colWidths=[200, 330])
    #     t.setStyle(TableStyle([('VALIGN', (0, 0), (1, 0), 'TOP')]))
    #     story.append(t)
    # story.append(Spacer(1, 8))


#    story.append( Paragraph( text_weights, courier_text) )
#    story.append(Spacer(1, 8))

    text_flight_crew_details = '''
        FLIGHT CREW DETAILS <br />
        =================== <br />
        '''
    # for crew_detail in d['pilot']:

    #     text_flight_crew_details += '''
    #     {} <br />
    #     &nbsp;&nbsp;&nbsp;&nbsp; ID: {}<br />
    #     &nbsp;&nbsp;&nbsp;&nbsp; DOB: {}<br />
    #     &nbsp;&nbsp;&nbsp;&nbsp; ROLE: {}<br />
    #     '''.format(
    #         crew_detail['name'],
    #         crew_detail['id'],
    #         crew_detail['dob'],
    #         crew_detail['role']
    #     )

    # story.append(Paragraph(text_flight_crew_details, courier_text))
    # story.append(Spacer(1, 8))

    mission, contingency, final = calculate_charge()

    for sector_id in range(len(d['sector_details'])):

        text_flight_plan = '''
            FLIGHT PLAN <br />
            =========== <br />
            Sector Number: {}  <br />
            {}                CHARGE{}   TIME <br />
            MISSION{}         {}%{}   {} mins <br />
            CONTINGENCY{}     {}%{}   {} mins <br />
            FINAL{}           {}%{}   {} mins <br />
            --------------------------------- <br />
            MIN T/O CHARGE{}  {}%{}   {} mins <br />
            '''.format(
            sector_id+1,
            '&nbsp;' * 14,
            '&nbsp;' * 4,
            '&nbsp;' * 10,
            mission[0],
            '&nbsp;' * 4,
            mission[1],
            '&nbsp;' * 7,
            contingency[0],
            '&nbsp;' * 5,
            contingency[1],
            '&nbsp;' * 12,
            final[0],
            '&nbsp;' * 4,
            final[1],
            '&nbsp;' * 3,
            mission[0] + contingency[0] + final[0],
            '&nbsp;' * 4,
            mission[1] + contingency[1] + final[1],
        )

    #    story.append( Paragraph( text_flight_plan, courier_text) )
    #    story.append(Spacer(1, 8))

        flight_details = '''
            {}  Time{}  Coordinates{}                Charge<br /><br />
            T/O ....... ................................... .......<br /><br />
            LDG ....... ................................... .......<br /><br />
            Pilot ............................ Battery used ....... <br />
            '''.format(
            '&nbsp;' * 3,
            '&nbsp;' * 10,
            '&nbsp;' * 17
        )

    #    story.append( Paragraph(flight_details, courier_text) )
    #    story.append(Spacer(1, 8))

        t = Table([(Paragraph(text_flight_plan, courier_text), Paragraph(
            flight_details, courier_text))], colWidths=[200, 330])
        # story.append(t)

    return story


# ========================================================================


def _drone():
    pass


def _weather(story, d):

    #    nicosia_fir = 'LCCC'
    #    airports = 'LCLK,LCPH'
    #
    #    #url_sigmet =
    #    url_metar  = 'https://www.aviationweather.gov/adds/dataserver_current/httpparam?datasource=metars&requestType=retrieve&format=xml&mostRecentForEachStation=constraint&hoursBeforeNow=1.25&stationString=LCLK LCPH'
    #    url_taf    = 'https://www.aviationweather.gov/adds/dataserver_current/httpparam?datasource=tafs&requestType=retrieve&format=xml&mostRecentForEachStation=true&hoursBeforeNow=2&stationString=LCLK LCPH'
    #
    #    xml_download_metar = requests.get(url_metar, allow_redirects=True)
    #    xml_download_taf   = requests.get(url_taf,   allow_redirects=True)

    #    with open('metar.xml', 'wb') as file:
    #        file.write(xml_download_metar.content)
    #    with open('taf.xml', 'wb') as file:
    #        file.write(xml_download_taf.content)

    #    tree = ET.parse('metar.xml')
    #    root_metar = tree.getroot()

    #    metar_LCLK = root_metar[6][0][0].text
    #    metar_LCPH = root_metar[6][1][0].text

    #    tree = ET.parse('taf.xml')
    #    root_taf = tree.getroot()

    #    taf_LCLK = root_taf[6][0][0].text
    #    taf_LCPH = root_taf[6][1][0].text
    #
    #
    #    story.append( Paragraph( 'METAR ' + metar_LCLK, courier_text) )
    #    story.append( Paragraph( taf_LCLK, courier_text) )
    #    story.append(Spacer(1, 8))
    #    story.append( Paragraph( 'METAR ' + metar_LCPH, courier_text) )
    #    story.append( Paragraph( taf_LCPH, courier_text) )
    #    #story.append(Spacer(1, 8))
    story.append(Paragraph(d['metar_LCLK'], courier_text))
    story.append(Paragraph(d['taf_LCLK'], courier_text))
    story.append(Spacer(1, 8))
    story.append(Paragraph(d['metar_LCPH'], courier_text))
    story.append(Paragraph(d['taf_LCPH'], courier_text))
    #story.append(Spacer(1, 8))

    # NICOSIA FIR Forecast (issued at 0400, 1000, 1600 and 2200 UTC)
    # Local Area Forecast (issued at 0230, 0830, 1430 and 2030 UTC)
    nicosia_fir_forecast = _wx2pdf(
        'https://www.dom.org.cy/FORECAST/fir.html', d['source']+'fir.pdf')
    local_area_forecast = _wx2pdf(
        'https://www.dom.org.cy/FORECAST/laf.html', d['source']+'laf.pdf')

    # x1, y1, x2, y2
    co1 = [0.0, 0.0, 595.0, 842.0]
    co2 = [0.0, 0.0, 595.0, 842.0]

    fir = PdfImage(nicosia_fir_forecast, width=450, height=450 *
                   1.414, kind='direct', coordinates=co1)
    #fir.drawHeight -= 200
    story.append(fir)
    laf = PdfImage(local_area_forecast, width=450, height=450 *
                   1.414, kind='direct', coordinates=co2)
    #laf.drawHeight -= 100
    story.append(laf)

    return story

# ========================================================================


def _notams(story, d):

    # ********** TO DELETE **********
    #    with open('notams_cyprus_210302.txt') as f:
    #        notams = json.load(f)

    #notams, indexes = get_notams(d)

    #story = []
    if not d['notam_error']:
        for notam in d['notams']:
            x = notam
            c = x.replace('\n', '<br />&nbsp;&nbsp;&nbsp;&nbsp;')
            #c = '<p />' + c + '<p />'
            story.append(Paragraph(c, courier_text))
            story.append(Spacer(1, 8))
    #        story.append(PageBreak())
    else:
        story.append(Paragraph(d['notams'], courier_text))

    return story

# ========================================================================


def create_pdf(d, path):
    # Creating a page template
    doc = BaseDocTemplate(
        filename=default_storage.path(path),
        pagesize=A4,
        title=None,
        author=None
    )

    notams_text_frame = Frame(
        x1=40,  # From left
        y1=60,  # From bottom
        height=page_height - 2*60,
        width=page_width-2*40,
        leftPadding=0,
        bottomPadding=0,
        rightPadding=0,
        topPadding=0,
        showBoundary=0,
        id='text_frame'
    )

    notams_page = PageTemplate(
        id='NotamsPage',
        frames=[notams_text_frame]
    )

    story = []

    _operations(story, d)
    story.append(PageBreak())

    _weather(story, d)
    story.append(PageBreak())

    _notams(story, d)

    # Adding the story to the template and template to the document
    doc.addPageTemplates(notams_page)

    doc.build(story)            # Building doc


# create_pdf()


def haversine(original, notams):

    #	print(original)
    #	print(notams)

    original = original / 180 * np.pi
    notams = notams / 180 * np.pi

    # print('notams: ', notams[0,:])

    dlat = notams[0] - original[0]
    dlon = notams[1] - original[1]

    a = np.sin(dlat/2)**2 + \
        np.cos(original[0]) * np.cos(notams[0]) * np.sin(dlon/2)**2
    distance = 2 * 6371 * np.arcsin(a**0.5)
    return distance


def get_weather(d):
    nicosia_fir = 'LCCC'
    airports = 'LCLK,LCPH'

    # url_sigmet =
    url_metar = 'https://www.aviationweather.gov/adds/dataserver_current/httpparam?datasource=metars&requestType=retrieve&format=xml&mostRecentForEachStation=constraint&hoursBeforeNow=1.25&stationString=LCLK LCPH'
    url_taf = 'https://www.aviationweather.gov/adds/dataserver_current/httpparam?datasource=tafs&requestType=retrieve&format=xml&mostRecentForEachStation=true&hoursBeforeNow=2&stationString=LCLK LCPH'

    xml_download_metar = requests.get(url_metar, allow_redirects=True)
    xml_download_taf = requests.get(url_taf,   allow_redirects=True)

    with open(d['source'] + 'metar.xml', 'wb') as file:
        file.write(xml_download_metar.content)
    with open(d['source'] + 'taf.xml', 'wb') as file:
        file.write(xml_download_taf.content)

    tree = ET.parse(d['source'] + 'metar.xml')
    root_metar = tree.getroot()

    metar_LCLK = root_metar[6][0][0].text
    metar_LCPH = root_metar[6][1][0].text

    tree = ET.parse(d['source'] + 'taf.xml')
    root_taf = tree.getroot()

    taf_LCLK = root_taf[6][0][0].text
    taf_LCPH = root_taf[6][1][0].text

    d.update({
        'metar_LCLK': 'METAR ' + metar_LCLK,
        'taf_LCLK': taf_LCLK,
        'metar_LCPH': 'METAR ' + metar_LCPH,
        'taf_LCPH': taf_LCPH
    })

    return d


def get_notams(d):

    lat = d['lat']
    lon = d['lon']
    start_datetime = d['op_start']
    end_datetime = d['op_end']
    max_altitude = d['max_altitude']

    # lat = 35
    # lon = 32
    # start_datetime = datetime.datetime.now()
    # end_datetime = datetime.datetime.now()
    # max_altitude = 400

    operational_radius = 2
    operational_buffer = 5

    regex_location = "/\d{3}/\d{3}/\d{4}[NS]\d{5}[EW]\d{3}"
    regex_start = "B\) \d{10}"
    regex_end = "C\) \d{10}"
    original_point = np.array((float(lat), float(lon)))

    filename = datetime.now().strftime('%Y-%m-%d') + '.json'
    # print(filename)

    try:
        if path.exists(d['source']+filename):
            with open(d['source']+filename) as f:
                notams = json.load(f)
        else:
            #		https://applications.icao.int/dataservices/apis.html
            #			url = 'https://v4p4sz5ijk.execute-api.us-east-1.amazonaws.com/anbdata/states/notams/notams-list?format=json&api_key=c692bc20-7ad5-11eb-bd95-d9a31e532e02&states=CYP'

            url = 'https://applications.icao.int/dataservices/api/notams-realtime-list?api_key=c692bc20-7ad5-11eb-bd95-d9a31e532e02&format=json&criticality=&locations=LCCC'

            notam_download = requests.get(url, allow_redirects=True)

            with open(d['source']+filename, 'wb') as file:
                file.write(notam_download.content)

            with open(d['source']+filename) as f:
                notams = json.load(f)
        indexes = []
        for i, notam in enumerate(notams):
            notam_all = notam['all']

            location = re.findall(regex_location, notam_all)[0]

            start = re.findall(regex_start, notam_all)[0]
            end = re.findall(regex_end, notam_all)
            notam_start_datetime = datetime(
                year=int('20' + start[3:5]),
                month=int(start[5:7]),
                day=int(start[7:9]),
                hour=int(start[9:11]),
                minute=int(start[11:13]),
                second=0
            )

            if end == []:
                notam_end_datetime = datetime.max
            # notam_end_datetime = datetime.max.strftime("%Y-%m-%d %H:%M:%S")
            else:
                end = end[0]
                notam_end_datetime = datetime(
                    year=int('20' + end[3:5]),
                    month=int(end[5:7]),
                    day=int(end[7:9]),
                    hour=int(end[9:11]),
                    minute=int(end[11:13]),
                    second=0
                )
            min_alt = int(location[1:4])
            max_alt = int(location[5:8])
            latitude = float(location[9:11]) + float(location[11:13]) / 60
            longitude = float(location[14:17]) + \
                float(location[17:19]) / 60
            distance = int(location[20:23])
            h_distances = haversine(
                original_point, np.array([latitude, longitude]))
            # h_distances = h_distances - distances - operational_radius
            loc_good = 0
            datetime_good = 0
            alt_good = 0
            if h_distances - distance - operational_radius < 0:
                loc_good = 1

            if start_datetime < notam_end_datetime or notam_start_datetime < end_datetime:
                datetime_good = 1
            if min_alt < 15:
                alt_good = 1
            if loc_good and datetime_good and alt_good:
                indexes.append(i)
            # print(notams)
        new_notams = [notams[i]['all'] for i in indexes]
        d.update({
            'notam_error': None,
            'notams': new_notams,
            'notam_indexes': str(indexes)
        })
    except:
        notam_error = '''
			Error producing NOTAMs
			\n Please visit:
			\n http://www.mcw.gov.cy/mcw/DCA/AIS/ais.nsf/All/EE69411919A34B0CC2257D5C001C503E?OpenDocument
			\n or
			\n https://www.notams.faa.gov/dinsQueryWeb/
			\n using Nicosia FIR LCCC
			'''
    return d


def get_data(d):

    d = get_weather(d)
    d = get_notams(d)

    return d


def main(user, drone, operation_name, latitude, longitude, altitude, radius, buffer_altitude, buffer_radius, start_date, end_date, path):

    if not os.path.exists(default_storage.path('daily_fly_notams')):
        os.makedirs(default_storage.path('daily_fly_notams'))
    data_input = {
        'source': default_storage.path('daily_fly_notams')+'/',
        'pilot_id': user,
        'drone_id': drone,
        'lat': latitude,
        'lon': longitude,
        'max_altitude': altitude,
        'operational_radius': radius,
        'buffer_radius': buffer_altitude,
        'buffer_altitude': buffer_radius,
        'op_start': start_date,
        'op_end': end_date,
        'est_flight_time': None,
        'type': 'type',
        'drone_id': drone,
        'drone_info': ['no'],
        #        'sector_details' : ( (drone_id1, flight_time1), (drone_id2, flight_time2) )
        'sector_details': [],
        'notam_file': '/file/notam.txt',
        'operation': operation_name,
    }
    data_input = get_data(data_input)
    pdf = create_pdf(data_input, path)
    return pdf


if __name__ == '__main__':
    main()
