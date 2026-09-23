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
    if d.get('notam_error') is None:
        # No error occurred
        if d.get('notams') and len(d['notams']) > 0:
            # We have NOTAMs to display
            story.append(Paragraph('RELEVANT NOTAMs:', courier_text))
            story.append(Spacer(1, 8))
            for notam in d['notams']:
                # Ensure notam is a string
                if isinstance(notam, str):
                    x = notam
                else:
                    x = str(notam)  # Convert to string if it's not already
                c = x.replace('\n', '<br />&nbsp;&nbsp;&nbsp;&nbsp;')
                #c = '<p />' + c + '<p />'
                story.append(Paragraph(c, courier_text))
                story.append(Spacer(1, 8))
        else:
            # No NOTAMs found that match criteria
            story.append(Paragraph('NO RELEVANT NOTAMs FOUND FOR THIS OPERATION', courier_text))
            story.append(Spacer(1, 8))
    #        story.append(PageBreak())
    else:
        # Error occurred while fetching NOTAMs
        story.append(Paragraph('NOTAM ERROR:', courier_text))
        story.append(Spacer(1, 8))
        # Handle the case where d['notams'] might be a list or string
        if isinstance(d['notams'], list):
            for error_msg in d['notams']:
                story.append(Paragraph(str(error_msg), courier_text))
        else:
            story.append(Paragraph(str(d['notams']), courier_text))

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
    url_metar = 'https://aviationweather.gov/api/data/metar?ids=LCLK%2CLCPH&hours=0&order=id%2C-obs&sep=true'
    url_taf = 'https://aviationweather.gov/api/data/taf?ids=LCLK%2CLCPH&hours=0&order=id%2C-obs&sep=true'

    try:
        # Download METAR data
        metar_response = requests.get(url_metar, allow_redirects=True)
        taf_response = requests.get(url_taf, allow_redirects=True)

        # Save raw text data for debugging
        with open(d['source'] + 'metar.txt', 'wb') as file:
            file.write(metar_response.content)
        with open(d['source'] + 'taf.txt', 'wb') as file:
            file.write(taf_response.content)

        # Parse METAR data (plain text format)
        metar_text = metar_response.text.strip()
        metar_lines = [line.strip() for line in metar_text.split('\n') if line.strip()]
        
        # Find METAR data for each airport
        metar_LCLK = ""
        metar_LCPH = ""
        
        for line in metar_lines:
            if line.startswith('LCLK'):
                metar_LCLK = line
            elif line.startswith('LCPH'):
                metar_LCPH = line

        # Parse TAF data (plain text format)
        taf_text = taf_response.text.strip()
        taf_lines = [line.strip() for line in taf_text.split('\n') if line.strip()]
        
        # Find TAF data for each airport (TAF can span multiple lines)
        taf_LCLK = ""
        taf_LCPH = ""
        current_taf = ""
        current_airport = ""
        
        for line in taf_lines:
            if line.startswith('TAF LCLK'):
                if current_taf and current_airport == 'LCLK':
                    taf_LCLK = current_taf.strip()
                current_taf = line
                current_airport = 'LCLK'
            elif line.startswith('TAF LCPH'):
                if current_taf and current_airport == 'LCLK':
                    taf_LCLK = current_taf.strip()
                current_taf = line
                current_airport = 'LCPH'
            else:
                # This is a continuation line
                if current_taf:
                    current_taf += " " + line
        
        # Don't forget the last TAF
        if current_taf and current_airport == 'LCPH':
            taf_LCPH = current_taf.strip()
        elif current_taf and current_airport == 'LCLK' and not taf_LCLK:
            taf_LCLK = current_taf.strip()

        d.update({
            'metar_LCLK': 'METAR ' + metar_LCLK if metar_LCLK else 'METAR LCLK - No data available',
            'taf_LCLK': taf_LCLK if taf_LCLK else 'TAF LCLK - No data available',
            'metar_LCPH': 'METAR ' + metar_LCPH if metar_LCPH else 'METAR LCPH - No data available',
            'taf_LCPH': taf_LCPH if taf_LCPH else 'TAF LCPH - No data available'
        })

    except Exception as e:
        # Fallback in case of any errors
        d.update({
            'metar_LCLK': f'METAR LCLK - Error retrieving data: {str(e)}',
            'taf_LCLK': f'TAF LCLK - Error retrieving data: {str(e)}',
            'metar_LCPH': f'METAR LCPH - Error retrieving data: {str(e)}',
            'taf_LCPH': f'TAF LCPH - Error retrieving data: {str(e)}'
        })

    return d


def get_notams(d):

    lat = d['lat']
    lon = d['lon']
    start_datetime = d['op_start']
    end_datetime = d['op_end']
    max_altitude = d['max_altitude']

    operational_radius = 2
    operational_buffer = 5
    original_point = np.array((float(lat), float(lon)))

    filename = datetime.now().strftime('%Y-%m-%d') + '.json'

    try:
        if path.exists(d['source']+filename):
            with open(d['source']+filename) as f:
                notams = json.load(f)
        else:
            url = 'https://applications.icao.int/dataservices/api/notams-realtime-list?api_key=c692bc20-7ad5-11eb-bd95-d9a31e532e02&format=json&criticality=&locations=LCCC'

            notam_download = requests.get(url, allow_redirects=True)

            with open(d['source']+filename, 'wb') as file:
                file.write(notam_download.content)

            with open(d['source']+filename) as f:
                notams = json.load(f)
        
        # Debug: Check if notams is actually a list
        if not isinstance(notams, list):
            raise ValueError("Expected list but got " + str(type(notams)) + ": " + str(notams)[:200] + "...")
                
        indexes = []
        relevant_notams = []
        
        # Simplified approach - just include NOTAMs that are active during operation time
        for i, notam in enumerate(notams):
            try:
                # Get basic info safely
                notam_id = str(notam.get('id', 'N/A'))
                notam_message = str(notam.get('message', 'No message available'))
                notam_start_str = str(notam.get('startdate', ''))
                notam_end_str = str(notam.get('enddate', ''))
                
                # Simple time check
                time_relevant = True  # Default to include if we can't parse dates
                
                if notam_start_str and notam_end_str:
                    try:
                        notam_start = datetime.fromisoformat(notam_start_str.replace('Z', '+00:00')).replace(tzinfo=None)
                        notam_end = datetime.fromisoformat(notam_end_str.replace('Z', '+00:00')).replace(tzinfo=None)
                        time_relevant = (start_datetime < notam_end and notam_start < end_datetime)
                    except:
                        time_relevant = True  # Include if date parsing fails
                
                # For now, include all time-relevant NOTAMs (we can add location filtering later)
                if time_relevant:
                    indexes.append(i)
                    formatted_notam = "NOTAM " + notam_id + " - " + notam_message
                    relevant_notams.append(formatted_notam)
                    
            except Exception as individual_error:
                # Skip individual NOTAMs with errors but continue processing
                continue
        
        # If no NOTAMs match filtering criteria, provide summary
        if len(relevant_notams) == 0:
            summary = ("Total NOTAMs downloaded: " + str(len(notams)) + 
                      ". None match operation time criteria.")
            relevant_notams = [summary]
            
        d.update({
            'notam_error': None,
            'notams': relevant_notams,
            'notam_indexes': str(indexes),
            'total_notams_downloaded': len(notams)
        })
        
    except Exception as e:
        notam_error = ('Error producing NOTAMs: ' + str(e) + 
                      ' Please visit: http://www.mcw.gov.cy/mcw/DCA/AIS/ais.nsf/All/EE69411919A34B0CC2257D5C001C503E?OpenDocument'
                      ' or https://www.notams.faa.gov/dinsQueryWeb/'
                      ' using Nicosia FIR LCCC')
        d.update({
            'notam_error': notam_error,
            'notams': [notam_error],
            'notam_indexes': '[]'
        })
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
